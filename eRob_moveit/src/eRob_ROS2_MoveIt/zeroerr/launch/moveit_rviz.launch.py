import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def _expand_cpu_list(value: str) -> set[int]:
    cpus: set[int] = set()
    for part in str(value or "").split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = [int(item) for item in part.split("-", 1)]
            cpus.update(range(start, end + 1))
        else:
            cpus.add(int(part))
    return cpus


def _compact_cpu_list(cpus: set[int]) -> str:
    if not cpus:
        return "0"
    values = sorted(cpus)
    ranges = []
    start = previous = values[0]
    for value in values[1:]:
        if value == previous + 1:
            previous = value
            continue
        ranges.append((start, previous))
        start = previous = value
    ranges.append((start, previous))
    return ",".join(str(a) if a == b else f"{a}-{b}" for a, b in ranges)


def _kernel_isolated_cores() -> str:
    try:
        with open("/sys/devices/system/cpu/isolated") as f:
            isolated = f.read().strip()
            if isolated:
                return isolated
    except Exception:
        pass
    try:
        with open("/proc/cmdline") as f:
            for token in f.read().split():
                if token.startswith("isolcpus=") or token.startswith("nohz_full="):
                    value = token.split("=", 1)[1]
                    for prefix in ("managed_irq,", "domain,", "nohz,"):
                        value = value.replace(prefix, "")
                    return value
    except Exception:
        pass
    return ""


def _default_non_rt_cores() -> str:
    all_cpus = set(range(os.cpu_count() or 1))
    isolated = _expand_cpu_list(_kernel_isolated_cores())
    non_rt = all_cpus - isolated
    return _compact_cpu_list(non_rt or all_cpus)


def _env_core_list(name: str, default: str) -> str:
    value = os.environ.get(name, "").strip()
    return value or default


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "eRobo3", package_name="zeroerr"
    ).to_moveit_configs()

    non_rt_cores = _env_core_list("ZEROERR_NON_RT_CORES", _default_non_rt_cores())
    non_rt_prefix = f"taskset -c {non_rt_cores}"

    ld = LaunchDescription()
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="log",
            respawn=False,
            prefix=non_rt_prefix,
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    )

    return ld
