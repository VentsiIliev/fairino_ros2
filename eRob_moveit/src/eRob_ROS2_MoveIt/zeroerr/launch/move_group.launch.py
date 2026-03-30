from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("eRobo3", package_name="zeroerr")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
