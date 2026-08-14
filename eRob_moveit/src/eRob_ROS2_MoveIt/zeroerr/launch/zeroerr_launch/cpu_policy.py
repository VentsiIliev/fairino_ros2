"""CPU policy helpers shared by ZeroErr launch files.

Launch Python only consumes the exported ZEROERR_* policy values sourced from
zeroerr_rt_policy.env by the launch scripts. It must not rediscover the policy
from the kernel CPU topology files or the kernel command line; zeroerr_rt_policy.env
is the single source of truth.
"""

import os
from dataclasses import dataclass


def env_value(name: str, default: str) -> str:
    value = os.environ.get(name, "").strip()
    return value or default


def env_bool(name: str, default: bool) -> bool:
    value = os.environ.get(name, "").strip().lower()
    if not value:
        return default
    return value in {"1", "true", "yes", "on"}


def env_float(name: str, default: float) -> float:
    value = os.environ.get(name, "").strip()
    if not value:
        return default
    try:
        return float(value)
    except ValueError:
        return default


@dataclass(frozen=True)
class CpuPolicy:
    non_rt_cores: str
    planner_cores: str
    low_priority_cores: str
    control_cores: str
    non_rt_prefix: str
    planner_prefix: str
    low_priority_prefix: str
    control_prefix: str


def load_cpu_policy() -> CpuPolicy:
    non_rt_cores = env_value("ZEROERR_NON_RT_CORES", "0")
    planner_cores = env_value("ZEROERR_PLANNER_CORES", non_rt_cores)
    low_priority_cores = env_value("ZEROERR_LOW_PRIORITY_CORES", non_rt_cores)
    control_cores = env_value("ZEROERR_CONTROL_CORES", "0")
    return CpuPolicy(
        non_rt_cores=non_rt_cores,
        planner_cores=planner_cores,
        low_priority_cores=low_priority_cores,
        control_cores=control_cores,
        non_rt_prefix=f"taskset -c {non_rt_cores}",
        planner_prefix=f"taskset -c {planner_cores}",
        low_priority_prefix=f"taskset -c {low_priority_cores} nice -n 19",
        control_prefix=f"taskset -c {control_cores}",
    )


@dataclass(frozen=True)
class ServoPolicy:
    low_cpu: bool
    period: float
    realtime: bool


def load_servo_policy(config: dict) -> ServoPolicy:
    low_cpu = env_bool("ZEROERR_SERVO_LOW_CPU", False)
    config_period = float(config.get("publish_period", 0.01))
    if low_cpu:
        period = env_float("ZEROERR_SERVO_PERIOD", 0.02)
        realtime = env_bool("ZEROERR_SERVO_REALTIME", False)
    else:
        period = env_float("ZEROERR_SERVO_PERIOD", config_period)
        realtime = env_bool("ZEROERR_SERVO_REALTIME", True)
    return ServoPolicy(low_cpu=low_cpu, period=period, realtime=realtime)
