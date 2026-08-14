import os
import sys

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from zeroerr_launch.moveit_config import build_moveit_config
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    package_path = get_package_share_directory("zeroerr")
    moveit_config = build_moveit_config("zeroerr", package_path)
    return generate_spawn_controllers_launch(moveit_config)
