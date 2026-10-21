"""Billet cell with the FANUC CRX-10iA/L (official driver).

Starts robot_state_publisher, ros2_control, the joint_state_broadcaster +
joint_trajectory_controller, move_group with the MTC ExecuteTaskSolution capability, and RViz.

    hw:=mock   FANUC mock hardware (default)
    hw:=isaac  Isaac Sim is the robot (start scripts/run_isaac.sh first); sim time + magnet_bridge
    hw:=real   real controller (robot_ip:=...)

Needs ws_fanuc sourced:  source ~/ws_fanuc/install/setup.bash
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

sys.path.append(os.path.dirname(__file__))
from crx_config import crx_moveit_config  # noqa: E402


def launch_setup(context):
    hw = LaunchConfiguration("hw").perform(context)
    isaac = hw == "isaac"
    moveit_config = crx_moveit_config(hw=hw)
    share = get_package_share_directory("fanuc_pick_drop")
    rviz_config = os.path.join(share, "config", "pick_drop.rviz")
    sim_time = {"use_sim_time": isaac}

    ros2_controllers = [
        ParameterFile(
            PathJoinSubstitution(
                [FindPackageShare("fanuc_hardware_interface"), "config", "ros2_controllers.yaml"]
            ),
            allow_substs=True,
        )
    ]
    if isaac:
        # FANUC's ScaledJointTrajectoryController needs the robot's status GPIOs; Isaac has none.
        ros2_controllers.append(os.path.join(share, "config", "isaac_controllers.yaml"))

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, *ros2_controllers, sim_time],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, sim_time],
    )
    spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager"],
        )
        for name in ("joint_state_broadcaster", "joint_trajectory_controller")
    ]
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            sim_time,
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim_time,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    nodes = [ros2_control, robot_state_publisher, *spawners, move_group, rviz]
    if isaac:
        nodes.append(
            Node(package="fanuc_pick_drop", executable="magnet_bridge", output="screen", parameters=[sim_time])
        )
    return nodes


def generate_launch_description():
    # The official controller config uses $(var ...) substitutions for prefix/gpio_*.
    return LaunchDescription(
        [
            DeclareLaunchArgument("hw", default_value="mock", description="mock | isaac | real"),
            DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
            DeclareLaunchArgument("prefix", default_value=""),
            DeclareLaunchArgument("gpio_config_package", default_value="fanuc_hardware_interface"),
            DeclareLaunchArgument("gpio_config_path", default_value="config/example_gpio_config.yaml"),
            OpaqueFunction(function=launch_setup),
        ]
    )
