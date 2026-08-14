"""MoveIt config builder helpers shared by ZeroErr launch files."""

from typing import Optional, Sequence, Union

from launch.substitution import Substitution
from moveit_configs_utils import MoveItConfigsBuilder

from zeroerr_launch.runtime_config import (
    moveit_controllers_path_from_runtime,
    srdf_path_from_runtime,
    urdf_path_from_runtime,
)

DEFAULT_PLANNING_PIPELINES = ["pilz_industrial_motion_planner", "ompl", "stomp"]


def build_moveit_config(
    package_name: str,
    package_path: str,
    *,
    use_fake_hardware: Union[str, bool, Substitution, None] = None,
    planning_pipelines: Optional[Sequence[str]] = None,
    default_planning_pipeline: Optional[str] = None,
):
    """Build the shared MoveItConfigs for the ZeroErr robot.

    :param package_name: MoveIt robot name used by MoveItConfigsBuilder.
    :param package_path: Absolute path to the zeroerr package share directory.
    :param use_fake_hardware: "true"/"false" string, bool, or a launch Substitution
        (e.g. a LaunchConfiguration) for runtime selection. A Substitution makes the
        robot_description xacro expand at node start with the resolved value, so the
        launch argument (not only the env default) controls the hardware mode.
    :param planning_pipelines: Planning pipeline names to load; defaults to the
        canonical list. Order and explicit default are preserved per caller.
    :param default_planning_pipeline: Explicit default pipeline. If None, the
        MoveItConfigsBuilder defaults to "ompl" when present, else the first pipeline.
    """
    urdf_path = urdf_path_from_runtime(package_path)
    srdf_path = srdf_path_from_runtime(package_path)
    moveit_controllers_path = moveit_controllers_path_from_runtime(package_path)

    mappings = {"robot_urdf": urdf_path}

    if use_fake_hardware is not None:
        if isinstance(use_fake_hardware, bool):
            mappings["use_fake_hardware"] = "true" if use_fake_hardware else "false"
        else:
            mappings["use_fake_hardware"] = use_fake_hardware

    return (
        MoveItConfigsBuilder("eRobo3", package_name=package_name)
        .robot_description(
            file_path="config/urdfs/eRobo3.urdf.xacro",
            mappings=mappings,
        )
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(
            file_path=moveit_controllers_path,
        )
        .planning_pipelines(
            default_planning_pipeline=default_planning_pipeline,
            pipelines=list(planning_pipelines or DEFAULT_PLANNING_PIPELINES),
        )
        .to_moveit_configs()
    )