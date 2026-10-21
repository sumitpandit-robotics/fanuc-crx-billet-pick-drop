"""Run the billet bin-picking task against a running sim.launch.py (or real robot) stack.

    ros2 launch fanuc_pick_drop pick_drop.launch.py billets:=5 execute:=false
    ros2 launch fanuc_pick_drop pick_drop.launch.py use_sim_time:=true   # with hw:=isaac
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.append(os.path.dirname(__file__))
from crx_config import crx_moveit_config  # noqa: E402


def generate_launch_description():
    moveit_config = crx_moveit_config()
    cell_params = os.path.join(
        get_package_share_directory("fanuc_pick_drop"), "config", "pick_drop.yaml"
    )

    pick_drop = Node(
        package="fanuc_pick_drop",
        executable="pick_drop",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            cell_params,
            {
                "billets": ParameterValue(LaunchConfiguration("billets"), value_type=int),
                "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
            },
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("billets", default_value="5", description="How many billets to move"),
            DeclareLaunchArgument("execute", default_value="true", description="false = plan only"),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="true with sim.launch.py hw:=isaac"),
            pick_drop,
        ]
    )
