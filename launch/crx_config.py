"""MoveIt config shared by sim.launch.py and pick_drop.launch.py.

CRX-10iA/L + Schmalz EOAT. Kinematics, joint limits, OMPL and controller settings come
from FANUC's official fanuc_moveit_config (ws_fanuc); robot description and SRDF from
this package.
"""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def crx_moveit_config(hw="mock"):
    """hw: mock | isaac | real (selects the ros2_control hardware in the URDF)."""
    share = get_package_share_directory("fanuc_pick_drop")
    return (
        MoveItConfigsBuilder("crx10ia_l", package_name="fanuc_moveit_config")
        .robot_description(
            file_path=os.path.join(share, "urdf", "crx10ia_l_schmalz.urdf.xacro"),
            mappings={"hw": hw},
        )
        .robot_description_semantic(file_path=os.path.join(share, "srdf", "crx10ia_l_schmalz.srdf"))
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
