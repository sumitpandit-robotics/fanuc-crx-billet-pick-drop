"""Billet cell in Isaac Sim 6: the CRX-10iA/L is the simulated robot, billets are physical.

Run through scripts/run_isaac.sh (it expands the xacro and sets up Isaac's ROS 2 libs):

    scripts/run_isaac.sh [--headless]

ROS 2 interface (what sim.launch.py hw:=isaac expects):
    /isaac_joint_states    sensor_msgs/JointState  published every physics step
    /isaac_joint_commands  sensor_msgs/JointState  position + velocity targets from TopicBasedSystem
    /clock                 rosgraph_msgs/Clock     sim time (use_sim_time:=true everywhere)
    /magnet                std_msgs/String         billet id = magnet ON (pull in + weld), "" = release

The cell (table, pallets, billets) is built from config/pick_drop.yaml, the same file the
MTC node uses, so the physical cell matches MoveIt's planning scene.
"""
import argparse
import hashlib
import math
import os
import struct
import time

import numpy as np
import yaml
from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--urdf", required=True, help="Expanded URDF with absolute mesh paths")
parser.add_argument("--share", required=True, help="fanuc_pick_drop share directory")
parser.add_argument("--cache", default=os.path.expanduser("~/.cache/fanuc_pick_drop/isaac"))
parser.add_argument("--headless", action="store_true")
parser.add_argument("--dt", type=float, default=1.0 / 120.0, help="Physics step [s]")
parser.add_argument("--rtf", type=float, default=1.0, help="Max real-time factor (0 = as fast as possible)")
parser.add_argument("--settle-test", type=float, metavar="SECONDS", help="No ROS: simulate, report billet drift, exit")
args, _ = parser.parse_known_args()

simulation_app = SimulationApp({"headless": args.headless})

import omni.kit.app  # noqa: E402

ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in ("isaacsim.ros2.bridge", "omni.scene.optimizer.core", "isaacsim.robot.schema"):
    ext_manager.set_extension_enabled_immediate(ext, True)
simulation_app.update()

import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
import omni.graph.core as og  # noqa: E402
import rclpy  # noqa: E402  (Isaac's bundled Humble rclpy, not the system one)
import usdrt.Sdf  # noqa: E402
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig  # noqa: E402
from isaacsim.core.experimental.prims import RigidPrim, XformPrim  # noqa: E402
from isaacsim.core.rendering_manager import RenderingManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from omni.physx import get_physx_simulation_interface  # noqa: E402
from omni.physx.bindings._physx import ContactEventType  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402

ROBOT_PATH = "/World/crx"
CELL_PATH = "/World/cell"
BILLETS_PATH = "/World/billets"
MAGNET_JOINT_PATH = "/World/magnet_joint"
STEEL_DENSITY = 7850.0  # kg/m^3
# Magnet capture range, billet centre relative to the TCP: across / along the billet, and the
# air gap between pole face and billet. Placeholder values; measure on the real gripper.
# Sideways: the 50 mm pole face still covers the billet's top line up to ~25 mm off-centre.
CAPTURE_SIDEWAYS = 0.020
CAPTURE_ALONG = 0.030
CAPTURE_GAP = 0.010


def load_cell(share):
    with open(os.path.join(share, "config", "pick_drop.yaml")) as f:
        return yaml.safe_load(f)["pick_drop"]["ros__parameters"]


def import_robot(stage):
    """URDF -> USD (cached by URDF contents), referenced at ROBOT_PATH. Returns the gripper body path."""
    with open(args.urdf, "rb") as f:
        tag = hashlib.sha1(f.read()).hexdigest()[:10]
    out_dir = os.path.join(args.cache, tag)
    name = os.path.splitext(os.path.basename(args.urdf))[0]
    usd = os.path.join(out_dir, name, f"{name}.usda")
    if not os.path.exists(usd):
        config = URDFImporterConfig(
            urdf_path=args.urdf,
            usd_path=out_dir,
            fix_base=True,
            merge_fixed_joints=False,  # keep tcp_link / schmalz_gripper_link as their own prims
            joint_drive_type="force",
            joint_target_type="position",
            # Stiff position servos, like the real controller. Isaac stores these per degree.
            override_joint_stiffness=1.0e5,
            override_joint_damping=1.0e3,
            robot_type="Manipulator",
        )
        usd = URDFImporter(config).import_urdf()
    stage_utils.add_reference_to_stage(usd, ROBOT_PATH)
    simulation_app.update()

    # A real robot controller compensates the arm's own weight. Without this, the position
    # drives sag ~1 mm at the tool under gravity, more than the 1 mm grasp standoff.
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(ROBOT_PATH):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            tight_contacts(prim)

    paths = {p.GetName(): str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()).startswith(ROBOT_PATH)}
    root = next(
        str(p.GetPath())
        for p in stage.Traverse()
        if str(p.GetPath()).startswith(ROBOT_PATH) and p.HasAPI(UsdPhysics.ArticulationRootAPI)
    )
    return root, paths["schmalz_gripper_link"], paths["tcp_link"]


def tight_contacts(prim, contact_offset=0.002):
    """Small explicit contact offset, zero rest offset: shapes only interact when they touch.

    PhysX's automatic contact offset is centimetres; around a 1 mm grasp standoff that let the
    hovering gripper nudge billets out of place before the magnet switched on.
    """
    api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
    api.CreateContactOffsetAttr(contact_offset)
    api.CreateRestOffsetAttr(0.0)


def physics_material(stage, path, friction, restitution=0.0):
    material = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(restitution)
    return material


def bind_material(prim, material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics"
    )


def read_stl(path):
    """Binary STL -> (points, face indices). The table mesh is ~250k triangles."""
    with open(path, "rb") as f:
        data = f.read()
    (count,) = struct.unpack_from("<I", data, 80)
    tris = np.frombuffer(data, dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]), count=count, offset=84)
    return tris["v"].reshape(-1, 3), np.arange(count * 3, dtype=np.int32)


def add_box(stage, path, size, center, material, color):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xform = UsdGeom.XformCommonAPI(cube)
    xform.SetTranslate(Gf.Vec3d(*center))
    xform.SetScale(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    bind_material(cube.GetPrim(), material)


def build_cell(stage, cell):
    """Table, two pallets (static) and billets (dynamic), mirroring PickDropNode::setupPlanningScene."""
    UsdGeom.Xform.Define(stage, CELL_PATH)
    UsdLux.DistantLight.Define(stage, "/World/light").CreateIntensityAttr(3000)
    UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(500)
    wood = physics_material(stage, "/World/materials/pallet", friction=0.6)
    steel = physics_material(stage, "/World/materials/steel", friction=0.6)

    # Static floor so a dropped billet can't fall forever
    add_box(stage, f"{CELL_PATH}/floor", (6, 6, 0.02), (0, 0, -0.01), wood, (0.35, 0.35, 0.35))

    # Table mesh (STL in mm), triangle-mesh collider (static, so no convex approximation needed)
    stl = cell["table_mesh"].replace("package://fanuc_pick_drop", args.share)
    points, indices = read_stl(stl)
    table = UsdGeom.Mesh.Define(stage, f"{CELL_PATH}/table")
    table.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points))
    table.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(indices) // 3, 3, dtype=np.int32)))
    table.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(indices))
    table.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.55, 0.6)])
    xform = UsdGeom.XformCommonAPI(table)
    xform.SetTranslate(Gf.Vec3d(*cell["table_position"]))
    xform.SetScale(Gf.Vec3f(0.001, 0.001, 0.001))
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(table.GetPrim()).CreateApproximationAttr("none")
    bind_material(table.GetPrim(), wood)

    # Pallets: floor + 4 walls, frame at the centre of the bottom face (as in makePallet)
    l, w, h = cell["pallet_size"]
    t, f, z0 = cell["pallet_wall"], cell["pallet_floor"], cell["table_top_z"]
    parts = {
        "floor": ((l, w, f), (0, 0, f / 2)),
        "wall_py": ((l, t, h), (0, w / 2 - t / 2, h / 2)),
        "wall_ny": ((l, t, h), (0, -w / 2 + t / 2, h / 2)),
        "wall_nx": ((t, w - 2 * t, h), (-l / 2 + t / 2, 0, h / 2)),
        "wall_px": ((t, w - 2 * t, h), (l / 2 - t / 2, 0, h / 2)),
    }
    for name, (x, y) in (("pick_pallet", cell["pick_pallet_xy"]), ("drop_pallet", cell["drop_pallet_xy"])):
        for part, (size, (dx, dy, dz)) in parts.items():
            add_box(stage, f"{CELL_PATH}/{name}/{part}", size, (x + dx, y + dy, z0 + dz), wood, (0.6, 0.45, 0.25))

    # Billets: steel cylinders lying along world X, resting on the pick pallet floor
    UsdGeom.Xform.Define(stage, BILLETS_PATH)
    r, length = cell["billet_radius"], cell["billet_length"]
    z = z0 + f + r
    for i, billet_id in enumerate(cell["billet_ids"]):
        x, y = cell["billet_xy"][2 * i : 2 * i + 2]
        cyl = UsdGeom.Cylinder.Define(stage, f"{BILLETS_PATH}/{billet_id}")
        cyl.CreateAxisAttr("X")
        cyl.CreateRadiusAttr(r)
        cyl.CreateHeightAttr(length)
        cyl.CreateDisplayColorAttr([Gf.Vec3f(0.7, 0.72, 0.75)])
        UsdGeom.XformCommonAPI(cyl).SetTranslate(Gf.Vec3d(x, y, z))
        prim = cyl.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        tight_contacts(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(STEEL_DENSITY * math.pi * r * r * length)
        # Cylinders on their side roll at the slightest touch; real billets barely do.
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateAngularDampingAttr(50.0)
        bind_material(prim, steel)


def build_ros_graph(articulation_root):
    """Same graph as Isaac's standalone_examples/api/isaacsim.ros2.bridge/moveit.py."""
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
                ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
                ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
                ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
                ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
                ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
                ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
                ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", articulation_root),
                ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(articulation_root)]),
                ("PublishJointState.inputs:topicName", "isaac_joint_states"),
                ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
            ],
        },
    )


def pose_matrix(path):
    """World pose of a prim as read from the running simulation (Isaac quaternions are w, x, y, z)."""
    positions, orientations = XformPrim(path).get_world_poses()
    p, (qw, qx, qy, qz) = positions.numpy()[0], orientations.numpy()[0]
    return Gf.Matrix4d().SetTransform(Gf.Rotation(Gf.Quatd(float(qw), float(qx), float(qy), float(qz))), Gf.Vec3d(*p.tolist()))


def pose_xyz_rpy(path):
    m = pose_matrix(path)
    rpy = m.ExtractRotation().Decompose(Gf.Vec3d.XAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.ZAxis())
    return m.ExtractTranslation(), rpy


class ContactMonitor:
    """Logs when a robot link starts touching something (other than the billet it carries).

    MoveIt checks its own collision model; this shows what physics actually hits. Billets don't
    report contacts: resting on a pallet they would report every step, and their convex-mesh
    cylinders make PhysX log a material warning for each report.
    """

    def __init__(self, stage, logger):
        self.logger, self.held = logger, ""
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.rsplit("/", 1)[-1] in ("world", "base_link", "J1_link"):
                continue  # bolted to / just above the table: constant contact
            if prim.HasAPI(UsdPhysics.RigidBodyAPI) and path.startswith(ROBOT_PATH):
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.on_contacts)

    @staticmethod
    def name(path):
        path = str(path)
        if path.startswith(CELL_PATH):
            return path[len(CELL_PATH) + 1 :]
        return path.rsplit("/", 1)[-1]

    def on_contacts(self, headers, data):
        for header in headers:
            if header.type != ContactEventType.CONTACT_FOUND:
                continue
            a = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
            robot = [p for p in (a, b) if p.startswith(ROBOT_PATH)]
            held = f"{BILLETS_PATH}/{self.held}" if self.held else None
            if robot and held in (a, b):
                continue  # the magnet is holding it
            if robot:
                where = ""
                if header.num_contact_data:
                    c = data[header.contact_data_offset]
                    where = f" at ({c.position[0]:.3f}, {c.position[1]:.3f}, {c.position[2]:.4f}), separation {c.separation * 1000:.1f} mm"
                self.logger.warn(f"contact: robot {self.name(a)} <-> {self.name(b)}{where}")


class Magnet(Node):
    """/magnet -> the billet is pulled onto the pole face and welded there (fixed joint).

    Like the real magnet, it only grabs a billet that is close: within the capture range the
    billet snaps flush and centred under the TCP; farther away, nothing is picked.
    """

    def __init__(self, stage, gripper_path, tcp_path, billet_radius):
        super().__init__("isaac_magnet")
        self.stage, self.gripper_path, self.tcp_path = stage, gripper_path, tcp_path
        self.radius = billet_radius
        self.held, self.request = "", None
        self.settling = {}  # released billet id -> physics steps until its resting pose is logged
        self.billet_ids = [p.GetName() for p in stage.GetPrimAtPath(BILLETS_PATH).GetChildren()]
        self.last = {b: pose_xyz_rpy(f"{BILLETS_PATH}/{b}")[0] for b in self.billet_ids}
        self.contacts = ContactMonitor(stage, self.get_logger())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "magnet", lambda msg: setattr(self, "request", msg.data), qos)

    def log_settled(self):
        """Where a released billet came to rest, to compare with its drop slot in pick_drop.yaml."""
        for billet_id in list(self.settling):
            self.settling[billet_id] -= 1
            if self.settling[billet_id] == 0:
                del self.settling[billet_id]
                (x, y, z), (roll, pitch, yaw) = pose_xyz_rpy(f"{BILLETS_PATH}/{billet_id}")
                self.get_logger().info(
                    f"{billet_id} resting at ({x:.3f}, {y:.3f}, {z:.3f}) m, rpy ({roll:.1f}, {pitch:.1f}, {yaw:.1f}) deg"
                )

    def log_moved(self):
        """Billets that moved since the last magnet event without being held: MoveIt doesn't know."""
        for b in self.billet_ids:
            now = pose_xyz_rpy(f"{BILLETS_PATH}/{b}")[0]
            if b != self.held and (now - self.last[b]).GetLength() > 0.002:
                self.get_logger().warn(f"{b} moved {(now - self.last[b]).GetLength() * 1000:.0f} mm on its own, now at ({now[0]:.3f}, {now[1]:.3f}, {now[2]:.3f})")
            self.last[b] = now

    def apply(self):
        """Called from the sim loop, so USD is only touched between physics steps."""
        self.log_settled()
        if self.request is None or self.request == self.held:
            self.request = None
            return
        billet_id, self.request = self.request, None
        self.log_moved()
        if self.stage.GetPrimAtPath(MAGNET_JOINT_PATH):
            self.stage.RemovePrim(MAGNET_JOINT_PATH)
            self.get_logger().info(f"Magnet OFF: released {self.held}")
            self.settling[self.held] = 120
            self.held = ""
            self.contacts.held = ""
        if not billet_id:
            return
        billet_path = f"{BILLETS_PATH}/{billet_id}"
        if not self.stage.GetPrimAtPath(billet_path):
            self.get_logger().error(f"Magnet ON: no billet {billet_path}")
            return
        gripper, tcp, billet = (pose_matrix(p) for p in (self.gripper_path, self.tcp_path, billet_path))
        # Billet centre in the TCP frame (row-vector convention: billet = in_tcp * tcp).
        # TCP +Z points out of the pole face into the billet, +Y runs along the billet.
        offset = (billet * tcp.GetInverse()).ExtractTranslation()
        gap = offset[2] - self.radius  # pole face to billet surface
        if abs(offset[0]) > CAPTURE_SIDEWAYS or abs(offset[1]) > CAPTURE_ALONG or not -0.002 < gap < CAPTURE_GAP:
            self.get_logger().error(
                f"Magnet ON: {billet_id} out of reach (centre offset x {offset[0] * 1000:.1f}, "
                f"y {offset[1] * 1000:.1f} mm, gap {gap * 1000:.1f} mm): nothing picked"
            )
            return
        # The magnet pulls the billet flush against the pole face, centred under the TCP with its
        # axis along TCP +Y. Billet axis is its local X: X_b = Y_tcp, Y_b = -X_tcp, Z_b = Z_tcp.
        in_tcp = Gf.Matrix4d().SetTransform(
            Gf.Rotation(Gf.Vec3d.ZAxis(), 90.0), Gf.Vec3d(0.0, 0.0, self.radius)
        )
        snapped = in_tcp * tcp
        position, rotation = snapped.ExtractTranslation(), snapped.ExtractRotationQuat()
        rigid = RigidPrim(billet_path)
        rigid.set_world_poses(
            positions=[list(position)],
            orientations=[[rotation.GetReal(), *rotation.GetImaginary()]],
        )
        rigid.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]])
        local = snapped * gripper.GetInverse()  # billet pose in the gripper body frame
        joint = UsdPhysics.FixedJoint.Define(self.stage, MAGNET_JOINT_PATH)
        joint.CreateBody0Rel().SetTargets([self.gripper_path])
        joint.CreateBody1Rel().SetTargets([billet_path])
        joint.CreateLocalPos0Attr(Gf.Vec3f(local.ExtractTranslation()))
        joint.CreateLocalRot0Attr(Gf.Quatf(local.ExtractRotationQuat()))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        self.held = billet_id
        self.contacts.held = billet_id
        self.get_logger().info(
            f"Magnet ON: {billet_id} pulled in by x {offset[0] * 1000:+.1f}, y {offset[1] * 1000:+.1f} mm, "
            f"gap {gap * 1000:.1f} mm"
        )


def log(text):
    print(f"[billet_cell] {text}", flush=True)


def main():
    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = stage_utils.get_current_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    # One physics step per app frame, so the ROS graph reads commands and publishes joint
    # states every step (the default 60 frames/s ran 2 physics steps per update).
    RenderingManager.set_dt(args.dt)
    UsdGeom.Xform.Define(stage, "/World")

    cell = load_cell(args.share)
    log("importing robot")
    articulation_root, gripper_path, tcp_path = import_robot(stage)
    log("building cell")
    build_cell(stage, cell)
    build_ros_graph(articulation_root)
    simulation_app.update()

    log("starting physics")
    SimulationManager.setup_simulation(dt=args.dt, device="cpu")
    rclpy.init()
    magnet = Magnet(stage, gripper_path, tcp_path, cell["billet_radius"])
    app_utils.play()
    log(f"running: robot {articulation_root}, gripper {gripper_path}")

    if args.settle_test:
        for _ in range(int(args.settle_test / args.dt)):
            simulation_app.update()
        magnet.log_moved()
        (x, y, z), rpy = pose_xyz_rpy(tcp_path)
        log(f"tcp_link at ({x:.4f}, {y:.4f}, {z:.4f}) rpy ({rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f})")
        log("settle test done")
        simulation_app.close()
        return

    # Pace to wall-clock time: ros2_control's update loop runs on wall-clock time.
    start, steps = time.monotonic(), 0
    while simulation_app.is_running():
        simulation_app.update()
        rclpy.spin_once(magnet, timeout_sec=0.0)
        magnet.apply()
        steps += 1
        if args.rtf > 0:
            ahead = start + steps * args.dt / args.rtf - time.monotonic()
            if ahead > 0:
                time.sleep(ahead)

    app_utils.stop()
    magnet.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
