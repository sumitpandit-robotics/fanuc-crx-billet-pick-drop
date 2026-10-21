"""Simulated billet cell with the FANUC CRX-10iA/L (official driver, mock hardware).

Starts robot_state_publisher, ros2_control (fanuc_hardware_interface mock), the
joint_state_broadcaster + joint_trajectory_controller, move_group with the MTC
ExecuteTaskSolution capability, and RViz.

Needs ws_fanuc sourced:  source ~/ws_fanuc/install/setup.bash
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

sys.path.append(os.path.dirname(__file__))
from crx_config import crx_moveit_config  # noqa: E402


def generate_launch_description():
    moveit_config = crx_moveit_config(use_mock="true")
    rviz_config = os.path.join(
        get_package_share_directory("fanuc_pick_drop"), "config", "pick_drop.rviz"
    )

    # The official controller config uses $(var ...) substitutions for these.
    args = [
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("gpio_config_package", default_value="fanuc_hardware_interface"),
        DeclareLaunchArgument("gpio_config_path", default_value="config/example_gpio_config.yaml"),
    ]
    ros2_controllers = ParameterFile(
        PathJoinSubstitution(
            [FindPackageShare("fanuc_hardware_interface"), "config", "ros2_controllers.yaml"]
        ),
        allow_substs=True,
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers],
        output="both",
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
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
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        args + [ros2_control, robot_state_publisher, *spawners, move_group, rviz]
    )
