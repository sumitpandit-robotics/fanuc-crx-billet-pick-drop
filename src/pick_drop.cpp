// Billet bin-picking for the FANUC CRX-10iA/L with the Schmalz SGM-HP magnetic EOAT.
//
// Cell (see config/pick_drop.yaml): table mesh, two open-top pallets, five steel billets
// (cylinders lying along world X) in the pick pallet. Each billet is moved to its slot in
// the drop pallet, one MTC task per billet.
//
// The EOAT has no moving joints, so "grip" and "release" are planning-scene changes:
// attaching the billet to tcp_link means magnet ON, detaching means magnet OFF.
// tcp_link sits at the centre of the pole face with +Z pointing out of it.
//
// Task layout (one billet):
//   current state
//   -> move to pick (OMPL)
//   -> pick:  approach down | IK on billet top | magnet ON | lift | clear of pallet
//   -> move to drop (OMPL, billet attached)
//   -> drop:  settle | lower | IK at drop slot | magnet OFF | retreat
//   -> return home (OMPL, after the last billet only)

#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/shape_operations.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include <thread>

namespace mtc = moveit::task_constructor;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_drop");

static constexpr char ARM_GROUP[] = "manipulator";
static constexpr char TCP_FRAME[] = "tcp_link";
static constexpr char HOME_STATE[] = "home";  // SRDF state: flange facing down
static constexpr char TABLE_ID[] = "workcell_table";
static constexpr char PICK_PALLET_ID[] = "pick_pallet";
static constexpr char DROP_PALLET_ID[] = "drop_pallet";
// Links that touch the billet while it is held (only links with collision geometry matter).
static const std::vector<std::string> GRIPPER_LINKS = { "schmalz_gripper_link" };

struct CellConfig
{
  std::string table_mesh;
  std::vector<double> table_position, pallet_size, pick_pallet_xy, drop_pallet_xy;
  double pallet_wall, pallet_floor, table_top_z;
  double billet_length, billet_radius;
  std::vector<std::string> billet_ids;
  std::vector<double> billet_xy, drop_xy;
  double clearance, grasp_standoff, approach_min, approach_max, lift_min, lift_max;
  int billets, planning_attempts;
  bool execute;

  double floorZ() const { return table_top_z + pallet_floor; }
  // Billet centre height when it rests on a pallet floor (pick) or is released (drop).
  double restingZ() const { return floorZ() + billet_radius; }
  double releaseZ() const { return restingZ() + clearance; }
};

// TCP orientation for a top-down grasp: +Z (pole-face normal) points straight down and +Y
// (the pole face's 121 mm long side) runs along the billet axis at angle `yaw` about world Z.
// Built as Rz(yaw + 90 deg) * Rx(180 deg).
static geometry_msgs::msg::Quaternion graspOrientation(double yaw)
{
  const double half = (yaw + M_PI_2) / 2.0;
  geometry_msgs::msg::Quaternion q;
  q.x = std::cos(half);
  q.y = std::sin(half);
  q.z = 0.0;
  q.w = 0.0;
  return q;
}

static geometry_msgs::msg::Vector3Stamped worldZ(double sign)
{
  geometry_msgs::msg::Vector3Stamped v;
  v.header.frame_id = "world";
  v.vector.z = sign;
  return v;
}

static geometry_msgs::msg::Pose makePose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;
  return p;
}

class PickDropNode
{
public:
  explicit PickDropNode(const rclcpp::NodeOptions& options)
    : node_{ std::make_shared<rclcpp::Node>("pick_drop", options) }
  {
    auto get = [this](const std::string& name, auto& out, const auto& fallback) {
      node_->get_parameter_or(name, out, fallback);
    };
    using Vec = std::vector<double>;
    get("table_mesh", cfg_.table_mesh, std::string("package://fanuc_pick_drop/models/table.stl"));
    get("table_position", cfg_.table_position, Vec{ -0.790, -0.761, -0.001 });
    get("pallet_size", cfg_.pallet_size, Vec{ 0.600, 0.400, 0.150 });
    get("pallet_wall", cfg_.pallet_wall, 0.008);
    get("pallet_floor", cfg_.pallet_floor, 0.010);
    get("table_top_z", cfg_.table_top_z, 0.959);
    get("pick_pallet_xy", cfg_.pick_pallet_xy, Vec{ 0.0, -0.5 });
    get("drop_pallet_xy", cfg_.drop_pallet_xy, Vec{ 0.0, 0.5 });
    get("billet_length", cfg_.billet_length, 0.200);
    get("billet_radius", cfg_.billet_radius, 0.025);
    get("billet_ids", cfg_.billet_ids, std::vector<std::string>{});
    get("billet_xy", cfg_.billet_xy, Vec{});
    get("drop_xy", cfg_.drop_xy, Vec{});
    get("clearance", cfg_.clearance, 0.002);
    get("grasp_standoff", cfg_.grasp_standoff, 0.001);
    get("approach_min", cfg_.approach_min, 0.100);
    get("approach_max", cfg_.approach_max, 0.155);
    get("lift_min", cfg_.lift_min, 0.180);
    get("lift_max", cfg_.lift_max, 0.255);
    get("billets", cfg_.billets, 5);
    get("planning_attempts", cfg_.planning_attempts, 5);
    get("execute", cfg_.execute, true);
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  {
    return node_->get_node_base_interface();
  }

  bool validConfig() const
  {
    const size_t n = cfg_.billet_ids.size();
    if (n == 0 || cfg_.billet_xy.size() != 2 * n || cfg_.drop_xy.size() != 2 * n)
    {
      RCLCPP_ERROR(LOGGER, "billet_ids (%zu), billet_xy (%zu) and drop_xy (%zu) must describe the same billets", n,
                   cfg_.billet_xy.size(), cfg_.drop_xy.size());
      return false;
    }
    return true;
  }

  bool setupPlanningScene()
  {
    moveit::planning_interface::PlanningSceneInterface psi;

    // A previous run may have stopped with a billet still attached: release it first.
    for (const auto& [id, attached] : psi.getAttachedObjects())
    {
      moveit_msgs::msg::AttachedCollisionObject detach;
      detach.object.id = id;
      detach.link_name = attached.link_name;
      detach.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      psi.applyAttachedCollisionObject(detach);
    }
    // Objects from the earlier single-box demo.
    psi.removeCollisionObjects({ "object", "table" });

    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // Table mesh (STL in mm)
    shapes::Mesh* mesh = shapes::createMeshFromResource(cfg_.table_mesh, Eigen::Vector3d::Constant(0.001));
    if (!mesh)
    {
      RCLCPP_ERROR(LOGGER, "Could not load table mesh %s", cfg_.table_mesh.c_str());
      return false;
    }
    shapes::ShapeMsg mesh_msg;
    shapes::constructMsgFromShape(mesh, mesh_msg);
    delete mesh;
    moveit_msgs::msg::CollisionObject table;
    table.id = TABLE_ID;
    table.header.frame_id = "world";
    table.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(mesh_msg));
    table.mesh_poses.push_back(makePose(cfg_.table_position[0], cfg_.table_position[1], cfg_.table_position[2]));
    table.operation = moveit_msgs::msg::CollisionObject::ADD;
    objects.push_back(table);

    objects.push_back(makePallet(PICK_PALLET_ID, cfg_.pick_pallet_xy));
    objects.push_back(makePallet(DROP_PALLET_ID, cfg_.drop_pallet_xy));

    for (size_t i = 0; i < cfg_.billet_ids.size(); ++i)
      objects.push_back(makeBillet(cfg_.billet_ids[i], cfg_.billet_xy[2 * i], cfg_.billet_xy[2 * i + 1]));

    psi.applyCollisionObjects(objects);
    RCLCPP_INFO(LOGGER, "Scene: table, 2 pallets, %zu billets", cfg_.billet_ids.size());
    return true;
  }

  bool run()
  {
    const int n = std::min<int>(cfg_.billets, cfg_.billet_ids.size());
    for (int i = 0; i < n; ++i)
    {
      const bool last = (i == n - 1);
      RCLCPP_INFO(LOGGER, "=== Billet %d/%d: %s (%.3f, %.3f) -> drop slot (%.3f, %.3f) ===", i + 1, n,
                  cfg_.billet_ids[i].c_str(), cfg_.billet_xy[2 * i], cfg_.billet_xy[2 * i + 1], cfg_.drop_xy[2 * i],
                  cfg_.drop_xy[2 * i + 1]);
      if (!moveBillet(i, last))
        return false;
    }
    return true;
  }

private:
  // Open-top crate: floor + 4 walls, positioned like workcell_scene_node.py.
  moveit_msgs::msg::CollisionObject makePallet(const std::string& id, const std::vector<double>& xy) const
  {
    const double l = cfg_.pallet_size[0], w = cfg_.pallet_size[1], h = cfg_.pallet_size[2];
    const double t = cfg_.pallet_wall, f = cfg_.pallet_floor, z0 = cfg_.table_top_z;

    moveit_msgs::msg::CollisionObject o;
    o.id = id;
    o.header.frame_id = "world";
    o.pose = makePose(xy[0], xy[1], z0);  // crate frame: centre of its bottom face
    auto add = [&o](double dx, double dy, double dz, double x, double y, double z) {
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions = { dx, dy, dz };
      o.primitives.push_back(box);
      o.primitive_poses.push_back(makePose(x, y, z));
    };
    add(l, w, f, 0, 0, f / 2);                       // floor
    add(l, t, h, 0, w / 2 - t / 2, h / 2);           // +Y wall
    add(l, t, h, 0, -w / 2 + t / 2, h / 2);          // -Y wall
    add(t, w - 2 * t, h, -l / 2 + t / 2, 0, h / 2);  // -X wall
    add(t, w - 2 * t, h, l / 2 - t / 2, 0, h / 2);   // +X wall
    o.operation = moveit_msgs::msg::CollisionObject::ADD;
    return o;
  }

  // Cylinder lying along world X, resting on the pick pallet floor.
  moveit_msgs::msg::CollisionObject makeBillet(const std::string& id, double x, double y) const
  {
    moveit_msgs::msg::CollisionObject o;
    o.id = id;
    o.header.frame_id = "world";
    o.pose = makePose(x, y, cfg_.restingZ());
    o.pose.orientation.y = M_SQRT1_2;  // cylinder axis Z -> world X (90 deg about Y)
    o.pose.orientation.w = M_SQRT1_2;
    shape_msgs::msg::SolidPrimitive cyl;
    cyl.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    cyl.dimensions = { cfg_.billet_length, cfg_.billet_radius };
    o.primitives.push_back(cyl);
    o.primitive_poses.push_back(makePose(0, 0, 0));
    o.operation = moveit_msgs::msg::CollisionObject::ADD;
    return o;
  }

  bool moveBillet(int i, bool last)
  {
    // Billets lie along X; the pole face can line up with them either way round.
    // Try the camera on the +X side first, then flipped.
    for (double yaw : { 0.0, M_PI })
    {
      task_ = createTask(i, yaw, last);
      try
      {
        task_.init();
      }
      catch (mtc::InitStageException& e)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "Task init failed:\n" << e);
        return false;
      }
      if (task_.plan(cfg_.planning_attempts))
        break;
      RCLCPP_WARN(LOGGER, "No plan with grasp yaw %.0f deg", yaw * 180.0 / M_PI);
      task_.printState();
    }
    if (task_.solutions().empty())
    {
      RCLCPP_ERROR(LOGGER, "Task planning failed. Open the 'Motion Planning Tasks' panel in RViz to see "
                           "which stage has no solutions.");
      return false;
    }

    // Solutions are sorted by cost; the first one is the cheapest.
    const auto& best = *task_.solutions().front();
    RCLCPP_INFO(LOGGER, "Planned %zu solution(s); best cost %.3f", task_.solutions().size(), best.cost());
    task_.introspection().publishSolution(best);

    if (!cfg_.execute)
    {
      RCLCPP_INFO(LOGGER, "execute:=false, not moving the robot");
      return true;
    }
    auto result = task_.execute(best);
    if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(LOGGER, "Execution failed with MoveIt error code %d", result.val);
      return false;
    }
    RCLCPP_INFO(LOGGER, "Billet moved");
    return true;
  }

  mtc::Task createTask(int i, double yaw, bool last)
  {
    const std::string& billet = cfg_.billet_ids[i];
    const double bx = cfg_.billet_xy[2 * i], by = cfg_.billet_xy[2 * i + 1];
    const double dx = cfg_.drop_xy[2 * i], dy = cfg_.drop_xy[2 * i + 1];

    // Things the held billet may touch while it is inside a pallet: it rests on the floor,
    // and neighbours placed end-to-end touch it. Checked again during the transfer.
    std::vector<std::string> neighbours = { PICK_PALLET_ID, DROP_PALLET_ID };
    for (const auto& other : cfg_.billet_ids)
      if (other != billet)
        neighbours.push_back(other);

    mtc::Task task;
    task.stages()->setName(billet);
    task.loadRobotModel(node_);
    task.setProperty("group", ARM_GROUP);
    task.setProperty("ik_frame", TCP_FRAME);

    // Free-space moves: OMPL. Short straight moves inside the pallets: Cartesian.
    auto sampling = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
    auto cartesian = std::make_shared<mtc::solvers::CartesianPath>();
    cartesian->setMaxVelocityScalingFactor(0.3);
    cartesian->setMaxAccelerationScalingFactor(0.3);
    cartesian->setStepSize(0.005);

    auto current = std::make_unique<mtc::stages::CurrentState>("current state");
    mtc::Stage* current_ptr = current.get();
    task.add(std::move(current));

    auto vertical = [&](const std::string& name, double sign, double min, double max) {
      auto stage = std::make_unique<mtc::stages::MoveRelative>(name, cartesian);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setIKFrame(TCP_FRAME);
      stage->setMinMaxDistance(min, max);
      stage->setDirection(worldZ(sign));
      stage->properties().set("marker_ns", name);
      return stage;
    };
    auto connect = [&](const std::string& name) {
      auto stage = std::make_unique<mtc::stages::Connect>(
          name, mtc::stages::Connect::GroupPlannerVector{ { ARM_GROUP, sampling } });
      stage->setTimeout(10.0);
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      return stage;
    };
    auto poseIK = [&](const std::string& name, double x, double y, double z, mtc::Stage* monitored) {
      geometry_msgs::msg::PoseStamped target;
      target.header.frame_id = "world";
      target.pose.position.x = x;
      target.pose.position.y = y;
      target.pose.position.z = z;
      target.pose.orientation = graspOrientation(yaw);

      auto gen = std::make_unique<mtc::stages::GeneratePose>(name + " pose");
      gen->setPose(target);
      gen->setMonitoredStage(monitored);
      gen->properties().set("marker_ns", name);

      auto ik = std::make_unique<mtc::stages::ComputeIK>(name + " IK", std::move(gen));
      ik->setMaxIKSolutions(8);
      ik->setMinSolutionDistance(1.0);
      // Total budget for all solutions. The default is the group's per-call IK timeout
      // (5 ms in fanuc_moveit_config), far too short to find 8 collision-free solutions.
      ik->setTimeout(1.0);
      ik->setIKFrame(TCP_FRAME);
      ik->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      ik->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      return ik;
    };
    // Pole-face height above a billet centre.
    const double grasp_dz = cfg_.billet_radius + cfg_.grasp_standoff;

    task.add(connect("move to pick"));

    // ---------------------------------------------------------------- pick
    mtc::Stage* attach_ptr = nullptr;
    {
      auto pick = std::make_unique<mtc::SerialContainer>("pick");
      task.properties().exposeTo(pick->properties(), { "group", "ik_frame" });
      pick->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });

      // Planned backwards from the grasp: the arm ends up descending onto the billet.
      pick->insert(vertical("approach", -1.0, cfg_.approach_min, cfg_.approach_max));
      pick->insert(poseIK("grasp", bx, by, cfg_.restingZ() + grasp_dz, current_ptr));
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("magnet ON (attach)");
        stage->allowCollisions(billet, GRIPPER_LINKS, true);
        stage->attachObject(billet, TCP_FRAME);
        stage->allowCollisions(billet, neighbours, true);
        attach_ptr = stage.get();
        pick->insert(std::move(stage));
      }
      pick->insert(vertical("lift", 1.0, cfg_.lift_min, cfg_.lift_max));
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("clear of pallet");
        stage->allowCollisions(billet, neighbours, false);
        pick->insert(std::move(stage));
      }
      task.add(std::move(pick));
    }

    task.add(connect("move to drop"));

    // ---------------------------------------------------------------- drop
    {
      auto drop = std::make_unique<mtc::SerialContainer>("drop");
      task.properties().exposeTo(drop->properties(), { "group", "ik_frame" });
      drop->properties().configureInitFrom(mtc::Stage::PARENT, { "group", "ik_frame" });
      {
        // Mirror of "clear of pallet": the billet may touch its new neighbours again.
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("settle into pallet");
        stage->allowCollisions(billet, neighbours, true);
        drop->insert(std::move(stage));
      }
      drop->insert(vertical("lower", -1.0, cfg_.lift_min, cfg_.lift_max));
      // The attach stage's scene already allows the neighbour contacts (see "magnet ON").
      drop->insert(poseIK("release", dx, dy, cfg_.releaseZ() + grasp_dz, attach_ptr));
      {
        auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("magnet OFF (detach)");
        stage->detachObject(billet, TCP_FRAME);
        stage->allowCollisions(billet, GRIPPER_LINKS, false);
        stage->allowCollisions(billet, neighbours, false);
        drop->insert(std::move(stage));
      }
      drop->insert(vertical("retreat", 1.0, cfg_.approach_min, cfg_.approach_max));
      task.add(std::move(drop));
    }

    if (last)
    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("return home", sampling);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setGoal(HOME_STATE);
      task.add(std::move(stage));
    }
    return task;
  }

  rclcpp::Node::SharedPtr node_;
  CellConfig cfg_;
  mtc::Task task_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto pick_drop = std::make_shared<PickDropNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &pick_drop]() {
    executor.add_node(pick_drop->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(pick_drop->getNodeBaseInterface());
  });

  bool ok = pick_drop->validConfig() && pick_drop->setupPlanningScene() && pick_drop->run();
  RCLCPP_INFO(LOGGER, ok ? "Bin picking finished" : "Bin picking FAILED");

  // Keep the node alive so the task stays inspectable in RViz; Ctrl+C to quit.
  spin_thread->join();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
