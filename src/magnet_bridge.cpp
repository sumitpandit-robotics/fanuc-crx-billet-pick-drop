// Magnet bridge: turns MoveIt's attach/detach into a magnet command for the simulator.
//
// MTC's "magnet ON" / "magnet OFF" stages attach and detach the billet in move_group's
// planning scene while the solution executes. This node polls that scene and publishes
//   /magnet (std_msgs/String) = id of the object attached to tcp_link, or "" when none
// whenever it changes (latched). Isaac Sim (isaac/billet_cell.py) welds or releases the
// billet on it; on the real robot this is where the FANUC DO for the magnet would go.

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using GetPlanningScene = moveit_msgs::srv::GetPlanningScene;

static constexpr char TCP_FRAME[] = "tcp_link";

class MagnetBridge : public rclcpp::Node
{
public:
  MagnetBridge() : Node("magnet_bridge")
  {
    magnet_pub_ = create_publisher<std_msgs::msg::String>("magnet", rclcpp::QoS(1).transient_local());
    scene_client_ = create_client<GetPlanningScene>("get_planning_scene");
    timer_ = create_wall_timer(50ms, [this] { poll(); });
    publish("");  // magnet starts OFF
  }

private:
  void poll()
  {
    if (pending_ || !scene_client_->service_is_ready())
      return;
    auto request = std::make_shared<GetPlanningScene::Request>();
    request->components.components = moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    pending_ = true;
    scene_client_->async_send_request(request, [this](rclcpp::Client<GetPlanningScene>::SharedFuture future) {
      pending_ = false;
      std::string attached;
      for (const auto& object : future.get()->scene.robot_state.attached_collision_objects)
        if (object.link_name == TCP_FRAME)
          attached = object.object.id;
      if (attached != state_)
        publish(attached);
    });
  }

  void publish(const std::string& id)
  {
    state_ = id;
    RCLCPP_INFO(get_logger(), id.empty() ? "Magnet OFF" : "Magnet ON: %s", id.c_str());
    std_msgs::msg::String msg;
    msg.data = id;
    magnet_pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr magnet_pub_;
  rclcpp::Client<GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string state_;
  bool pending_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MagnetBridge>());
  rclcpp::shutdown();
  return 0;
}
