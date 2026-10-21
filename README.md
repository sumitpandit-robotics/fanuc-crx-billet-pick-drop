# fanuc_pick_drop: CRX-10iA/L billet bin-picking

Learning guide to the project: big picture, then each layer and its files, then
hands-on exercises, and the Isaac Sim physics version (section 7).

**Quick start** (two terminals):
```bash
source ~/ws_fanuc/install/setup.bash && export ROS_LOCALHOST_ONLY=1
ros2 launch fanuc_pick_drop sim.launch.py                 # terminal 1
ros2 launch fanuc_pick_drop pick_drop.launch.py           # terminal 2 (billets:=N execute:=false)
```

**Dependencies** (ROS 2 Humble):
- MoveIt 2 + MoveIt Task Constructor (`ws_moveit2`)
- `moveit_resources_fanuc_moveit_config` providing `config/schmalz_eoat.urdf.xacro` and the Schmalz SGM-HP meshes
- FANUC's official ROS 2 stack (`ws_fanuc`, overlaying `ws_moveit2`): `fanuc_crx_description`,
  `fanuc_hardware_interface`, `fanuc_moveit_config`

Build: clone into `ws_moveit2/src`, then `colcon build --symlink-install --packages-select fanuc_pick_drop`.
- For `hw:=isaac` only: Isaac Sim 6.0 (pip install, e.g. `~/isaacsim-env-6`) and
  `topic_based_ros2_control` (`sudo apt install ros-humble-topic-based-ros2-control`, or build
  tag 0.2.0 from source in the workspace). See section 7.

## 1. The big picture (read this first)

Goal: a FANUC CRX-10iA/L with a Schmalz magnetic gripper moves 5 steel billets
(Ø50 × 200 mm) from a pick pallet to a drop pallet. Everything runs in simulation for now.

Four layers. Each one only talks to the layer below it:

```
 4. TASK        pick_drop node (MoveIt Task Constructor)  "what to do"
 3. PLANNING    move_group (MoveIt)                        "how to move without hitting anything"
 2. CONTROL     ros2_control + joint_trajectory_controller "follow this trajectory"
 1. HARDWARE    fanuc_hardware_interface (mock now, real robot later)
 0. MODEL       URDF (shape) + SRDF (meaning)              used by all the layers above
```

One full cycle, from start to finish:
1. `pick_drop` puts the table, pallets and billets into MoveIt's **planning scene**.
2. For each billet it builds an MTC **Task** and **plans all of it** before anything moves.
3. It sends the best solution to `move_group`'s `execute_task_solution` action.
4. `move_group` sends each trajectory segment to `joint_trajectory_controller` and applies
   the scene changes in between ("attach billet" = magnet ON).
5. The controller moves the (mock) joints; `joint_state_broadcaster` publishes `/joint_states`;
   `robot_state_publisher` turns them into TF for RViz.

## 2. Where the files are (three workspaces)

| Workspace | What it holds | Who wrote it |
|---|---|---|
| `~/ws_moveit2/src/fanuc_pick_drop/` | **Your application**: task node, cell config, CRX+gripper model, launch files | us |
| `~/ws_moveit2/src/moveit_resources/fanuc_*` | Schmalz gripper xacro + STL meshes (shared with modular_bin_pick_ws) | you + fixes |
| `~/ws_fanuc/src/` | FANUC official: CRX description, hardware interface, MoveIt config | FANUC |

`ws_fanuc` is built **on top of** `ws_moveit2`, so `source ~/ws_fanuc/install/setup.bash` gives you both.

```
fanuc_pick_drop/
├── urdf/crx10ia_l_schmalz.urdf.xacro   robot + gripper + mounting on table
├── srdf/crx10ia_l_schmalz.srdf         planning group, home pose, collision exceptions
├── config/pick_drop.yaml               THE CELL: table, pallets, billets, drop slots, distances
├── config/pick_drop.rviz               RViz layout with the MTC panel
├── models/table.stl                    your table mesh (mm)
├── launch/crx_config.py                one place that builds the MoveIt config
├── launch/sim.launch.py                layers 1-3 + RViz
├── launch/pick_drop.launch.py          layer 4 (the task)
├── src/pick_drop.cpp                   the task logic
├── src/magnet_bridge.cpp               hw:=isaac: planning-scene attach/detach -> /magnet
├── urdf/crx_isaac_ros2_control.xacro   hw:=isaac: ros2_control over Isaac joint topics
├── config/isaac_controllers.yaml       hw:=isaac: controller overrides
├── isaac/billet_cell.py                Isaac Sim 6 cell: robot, pallets, physical billets, magnet
└── scripts/run_isaac.sh                starts billet_cell.py with Isaac's ROS 2 libraries
```

## 3. Layer 0: the robot model

### URDF: what the robot *is* (links, joints, shapes)
`urdf/crx10ia_l_schmalz.urdf.xacro` builds this chain:
```
world → base_link (on table at -0.65, 0, 0.959) → J1…J6 links → flange → fanuc_flange
      → tool0 → eoat_mount_link → schmalz_gripper_link → tcp_link (magnet face centre)
                                                       → camera_link
```
- **Visual meshes** are what you see. **Collision shapes** (simple boxes) are what MoveIt checks.
  They're separate on purpose: boxes are fast and slightly padded to be safe.
- **TCP (`tcp_link`)**: the point that "does the work". It sits at the centre of the magnet
  face, z-axis pointing out of the face. Every pick and drop pose describes where *this
  frame* should go.
- Lesson we learned: the first collision boxes sat at the flange while the real gripper is
  30 cm away. MoveIt would have driven the real gripper into the table. Always check
  collision shapes in RViz (*Show Robot Collision*).

Try: `ros2 run tf2_ros tf2_echo world tcp_link` while the sim runs.

### SRDF: what the robot *means* (for planning)
`srdf/crx10ia_l_schmalz.srdf`:
- **Planning group** `manipulator` = chain `base_link → tcp_link`. Plans move this chain.
- **Named state** `home` = FANUC default (J5 = −90°, flange down).
- **disable_collisions** = the *Allowed Collision Matrix*: link pairs that are never checked.
  Only safe reasons: *Adjacent*, *Default* (overlap at home), *Never* (sampled, never touch).
  Never hand-write these; generate them:
  ```bash
  /opt/ros/humble/lib/moveit_setup_assistant/collisions_updater \
    --urdf robot.urdf --srdf bare.srdf --output out.srdf --trials 100000 --default
  ```
  Lesson: the old hand-written file disabled the gripper vs the base and J1–J4, which hid
  roughly 10,000 real collisions.

## 4. Layers 1–3: what `sim.launch.py` starts
| Node | Job |
|---|---|
| `ros2_control_node` | loads `crx_mock_ros2_control_macro` (mock hardware: joints follow commands perfectly) |
| `joint_trajectory_controller` | follows trajectories (FollowJointTrajectory action) |
| `joint_state_broadcaster` | publishes `/joint_states` |
| `robot_state_publisher` | URDF + joint states → TF |
| `move_group` | MoveIt: planning scene, IK (KDL), OMPL, execution, **plus `ExecuteTaskSolutionCapability`** so MTC solutions can run |
| `rviz2` | visualisation + "Motion Planning Tasks" panel |

Kinematics, joint limits, OMPL settings and the controller list come from FANUC's
`fanuc_moveit_config` via `launch/crx_config.py`. For the real robot you'd switch the
mock to `use_mock:=false robot_ip:=…`.

Lesson: two sims on the same network (ROS_DOMAIN_ID 0) talk to each other, which caused
fake "execution failed" errors. `export ROS_LOCALHOST_ONLY=1` in every terminal.

## 5. Layer 4: the task (`src/pick_drop.cpp`)

### 5a. Setup
- Reads every value from `config/pick_drop.yaml` (with defaults in code).
- `setupPlanningScene()`: adds the table mesh, 2 pallets (floor + 4 walls each, as boxes) and
  5 billets (cylinders rotated to lie along X). These are **collision objects** MoveIt now avoids.

### 5b. MTC in one paragraph
A Task is a pipeline of **stages**. Each stage produces partial solutions:
- **Generators** create states from nothing (`CurrentState`, `GeneratePose` + `ComputeIK`).
- **Propagators** extend a state forwards or backwards (`MoveRelative`, `ModifyPlanningScene`).
- **Connectors** join two states with free-space motion (`Connect` with OMPL).

MTC searches for complete chains and ranks them by **cost**. Nothing moves until a whole
pick → drop chain exists.

### 5c. The task for one billet
```
current state
→ move to pick            Connect (OMPL)
→ pick
    approach              MoveRelative down 10–15.5 cm (planned BACKWARDS from the grasp)
    grasp IK              GeneratePose at billet top + ComputeIK (timeout 1 s!)
    magnet ON             ModifyPlanningScene: attach billet to tcp_link, allow contacts
    lift                  MoveRelative up 18–25.5 cm (clears 9 cm pallet walls)
    clear of pallet       forbid billet↔pallet/neighbours again
→ move to drop            Connect (OMPL), billet attached and collision-checked
→ drop
    settle into pallet    allow contacts again (mirror of "clear")
    lower                 MoveRelative down
    release IK            pose 2 mm above floor (+1 mm standoff)
    magnet OFF            detach, forbid contacts
    retreat               MoveRelative up
→ return home             MoveTo "home" (last billet only)
```

### 5d. The subtle ideas
1. **Magnet ON/OFF = attach/detach.** The gripper has no joints, so "gripping" means telling
   MoveIt the billet now moves with `tcp_link`.
2. **Grasp orientation** `graspOrientation(yaw)` = Rz(yaw+90°)·Rx(180°): TCP z points down, and the
   magnet face's 121 mm side lies along the billet. If yaw 0° fails, it tries 180°.
3. **Allow/forbid symmetry.** Billets touch the floor and their end-to-end neighbours, so
   those contacts are *allowed* only while inside a pallet. "clear" and "settle" are exact
   mirrors, because MTC's `Connect` requires both ends to have identical scenes.
4. **Monitored stage.** The release pose generator takes its scene from the "magnet ON"
   stage, so it already knows the billet is attached.
5. **Flush geometry.** Collision boxes end exactly at the magnet face, plus a 1 mm standoff.
   Otherwise every grasp counts as "gripper in collision with billet".

### 5e. Bugs we hit (each one teaches something)
| Symptom | Real cause | Lesson |
|---|---|---|
| Part stuck to the side of the gripper | TCP was on the side face | Measure the TCP from CAD, then from the pendant |
| `eef in collision: object - gripper` | box padding stuck 0.5 mm past the face | Collision shapes must stop at the TCP plane |
| `cycles:=3` ignored | node-name YAML beats wildcard launch params | ROS 2 parameter precedence |
| "unknown goal response", stale joint states | two sims on domain 0 | ROS_LOCALHOST_ONLY / ROS_DOMAIN_ID |
| billet 3 "no IK found" | MTC IK budget = FANUC's 5 ms | `ik->setTimeout(1.0)` |
| probe IK gave sideways tool | Python Quaternion defaults to w = 1 | Always set all 4 quaternion fields |

## 6. Hands-on exercises (in order)
Setup for each: two terminals, `source ~/ws_fanuc/install/setup.bash; export ROS_LOCALHOST_ONLY=1`.

1. **Watch it:** `sim.launch.py`, then `pick_drop.launch.py billets:=1`. In RViz open
   *Motion Planning Tasks*, click each stage, and scrub through the 5 alternative solutions.
2. **Plan without moving:** `execute:=false`. Compare the costs of the solutions.
3. **See the collision model:** RViz → MotionPlanning → *Show Robot Collision*. Find the 4 gripper boxes.
4. **Follow the frames:** `tf2_echo world tcp_link` during a pick and check that z points down (−1).
5. **Change the cell:** edit `pick_drop.yaml`, e.g. `billet_radius: 0.020`. No rebuild is needed (symlink install).
6. **Break it on purpose:** comment out `ik->setTimeout(1.0)`, rebuild, and watch billet 3 fail. Read the
   stage printout and find the failing stage.
7. **Add a 6th billet:** append an id plus xy pairs to `billet_ids`, `billet_xy` and `drop_xy`, making sure it fits in the pallet.
8. **Regenerate the collision matrix** with `collisions_updater` and diff it against the SRDF.

## 7. Isaac Sim: physics and a simulated magnet

`hw:=isaac` swaps the mock hardware (layer 1) for Isaac Sim. MoveIt, MTC and `pick_drop.cpp`
are unchanged: Isaac only replaces "the robot". The billets become real rigid bodies (steel,
3.1 kg) that fall, roll and get pushed, so this is where cell problems show up that the mock hides.

```
pick_drop (MTC) -> move_group -> joint_trajectory_controller -> TopicBasedSystem (ros2_control)
                       |                                     /isaac_joint_commands | ^ /isaac_joint_states
                       v                                                           v |
         planning scene attach/detach -> magnet_bridge -> /magnet ------------> Isaac Sim (+ /clock)
```

Run (three terminals, same ROS_DOMAIN_ID / ROS_LOCALHOST_ONLY in all of them):
```bash
# 1. Isaac (first start imports the URDF to USD, ~2 min; cached in ~/.cache/fanuc_pick_drop)
export ROS_LOCALHOST_ONLY=1
ros2 run fanuc_pick_drop run_isaac.sh            # or: .../scripts/run_isaac.sh --headless
# 2. ROS side, once Isaac prints "[billet_cell] running"
source ~/ws_fanuc/install/setup.bash && export ROS_LOCALHOST_ONLY=1
ros2 launch fanuc_pick_drop sim.launch.py hw:=isaac
# 3. the task
ros2 launch fanuc_pick_drop pick_drop.launch.py use_sim_time:=true
```
`run_isaac.sh --settle-test 10` builds the cell, simulates 10 s without ROS and reports any billet
that moved: a quick check after editing `pick_drop.yaml`.

What the Isaac terminal tells you (read it, it's the point of the physical sim):
- `Magnet ON: ... pulled in by x .., y .. mm`: how far the billet was from where MoveIt expected.
- `<billet> moved N mm on its own`: a billet MoveIt thinks is static has moved.
- `<billet> resting at (...)`: where a dropped billet ended up; compare with `drop_xy`.
- `contact: robot <link> <-> <object>`: physics found a contact MoveIt didn't plan for.

### 7a. The magnet model
`magnet_bridge` polls move_group's planning scene and publishes the id of the object attached to
`tcp_link` (or `""`). Isaac then welds the billet to the gripper with a fixed joint. Like a real
magnet it only grabs a billet that's close: within the capture range (`CAPTURE_*` in
`billet_cell.py`, placeholders to measure on the real gripper) the billet is pulled flush and
centred under the TCP; farther away nothing is picked and the error says why. On the real robot,
`magnet_bridge` is where the FANUC DO for the magnet goes.

### 7b. Lessons from bringing it up
| Symptom | Real cause | Fix / lesson |
|---|---|---|
| Arm lags 20-120 mrad behind the trajectory | Only position was commanded, so the drive's damping braked every motion and hit the URDF effort limit | Command position **and velocity** (feed-forward), like a real servo |
| `/clock` at 69 Hz with 120 Hz physics | Isaac ran 2 physics steps per app frame | `RenderingManager.set_dt(dt)`: one step per frame |
| "Goal reached" while the arm was still off | FANUC's 0.05 rad goal tolerance is for the real controller | 0.002 rad goal tolerance in `isaac_controllers.yaml` |
| JTC reports 25+ mrad error but the path is right | The ROS<->Isaac loop adds ~24 ms; a pure delay doesn't change the path | Keep FANUC's 0.25 rad path tolerance |
| Billets roll across the pallet | PhysX has no rolling resistance | Angular damping on the billets |
| Billets next to each other drift 3-11 mm when one is picked | The drop/pick slots touch end-to-end (zero gap), so lifting one drags its neighbour | Capture range absorbs it in sim; for the real cell, consider a gap between slots |
| `failed to create domain` / spawners hang | CycloneDDS allows ~10 participants per domain on localhost | Raise `MaxAutoParticipantIndex` in `CYCLONEDDS_URI` if you add nodes |

Open question: a billet still creeps a few mm while the gripper hovers 1 mm above it, even though
the collision shapes don't touch. It looks like a PhysX contact-offset effect; the capture range
covers it, but it isn't fully explained yet.

## 8. Where the project goes next
1. Magnet I/O on the real robot: FANUC GPIO DO via `fanuc_gpio_controller` (which DO number?).
2. TCP calibration on the real CRX → update `tcp_xyz`/`tcp_rpy` in `schmalz_eoat.urdf.xacro`.
3. Billet poses from the Mech-Mind camera instead of `pick_drop.yaml`.
4. Real robot: `use_mock:=false robot_ip:=…`, slow speeds first.
5. Fix `modular_bin_pick_ws` for the corrected TCP, and commit everything to git.
