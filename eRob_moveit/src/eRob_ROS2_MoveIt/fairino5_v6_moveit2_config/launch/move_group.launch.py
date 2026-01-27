from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    # Build MoveIt config with only Pilz planner (disable CHOMP, OMPL, STOMP for 8x faster startup)
    moveit_config = (
        MoveItConfigsBuilder("fairino5_v6_robot", package_name="fairino5_v6_moveit2_config")
        .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])  # Only load Pilz
        .to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
