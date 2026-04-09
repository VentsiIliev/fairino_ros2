#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, List

import numpy as np


JOINT_NAMES = [f"Joint_{index}" for index in range(1, 7)]
DEFAULT_INPUT = Path(__file__).resolve().parents[1] / "data" / "torque_sensor_log.csv"
DEFAULT_MODEL = Path(__file__).resolve().parents[1] / "config" / "torque_sensor_model.json"
DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "collision_monitor_config.json"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Recommend per-joint external torque thresholds from logged residuals."
    )
    parser.add_argument("--input", default=str(DEFAULT_INPUT), help="Input CSV log path")
    parser.add_argument("--model", default=str(DEFAULT_MODEL), help="Learned torque model JSON path")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Collision monitor config JSON path")
    parser.add_argument(
        "--percentile",
        type=float,
        default=99.9,
        help="Absolute residual percentile used for threshold recommendation",
    )
    parser.add_argument(
        "--margin-scale",
        type=float,
        default=1.1,
        help="Multiplier applied to the recommended threshold",
    )
    parser.add_argument(
        "--min-threshold",
        type=float,
        default=1.0,
        help="Minimum threshold in Nm",
    )
    parser.add_argument(
        "--max-following-error",
        type=float,
        default=200.0,
        help="Discard rows with larger absolute following_error_actual",
    )
    parser.add_argument(
        "--max-abs-acceleration",
        type=float,
        default=20.0,
        help="Discard rows with larger absolute acceleration_rad_s2",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Write the recommended thresholds back into the collision monitor config",
    )
    return parser.parse_args()


def _safe_float(value: str) -> float | None:
    if value == "":
        return None
    try:
        out = float(value)
    except ValueError:
        return None
    if not np.isfinite(out):
        return None
    return out


def _load_model(model_path: Path) -> Dict[str, np.ndarray]:
    payload = json.loads(model_path.read_text(encoding="utf-8"))
    joints_payload = payload.get("joints", {})
    result: Dict[str, np.ndarray] = {}
    for joint_name in JOINT_NAMES:
        joint_payload = joints_payload.get(joint_name)
        if not isinstance(joint_payload, dict) or joint_payload.get("skipped"):
            continue
        coeffs = joint_payload.get("coefficients")
        if isinstance(coeffs, list) and len(coeffs) == 6:
            result[joint_name] = np.array(coeffs, dtype=float)
    if not result:
        raise SystemExit(f"No usable joints found in model: {model_path}")
    return result


def _predict(coeffs: np.ndarray, q: float, dq: float, ddq: float) -> float:
    features = np.array([1.0, np.sin(q), np.cos(q), dq, np.sign(dq), ddq], dtype=float)
    return float(coeffs @ features)


def _collect_residuals(
    csv_path: Path,
    model: Dict[str, np.ndarray],
    max_following_error: float,
    max_abs_acceleration: float,
) -> Dict[str, List[float]]:
    residuals: Dict[str, List[float]] = defaultdict(list)
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            joint = row.get("joint", "")
            if joint not in model:
                continue
            q = _safe_float(row.get("position_rad", ""))
            dq = _safe_float(row.get("velocity_rad_s", ""))
            ddq = _safe_float(row.get("acceleration_rad_s2", ""))
            tau = _safe_float(row.get("torque_sensor_nm", ""))
            following_error = _safe_float(row.get("following_error_actual", ""))
            if q is None or dq is None or ddq is None or tau is None:
                continue
            if following_error is not None and abs(following_error) > max_following_error:
                continue
            if abs(ddq) > max_abs_acceleration:
                continue
            predicted = _predict(model[joint], q, dq, ddq)
            residuals[joint].append(abs(tau - predicted))
    return residuals


def _recommend_thresholds(
    residuals: Dict[str, List[float]],
    percentile: float,
    margin_scale: float,
    min_threshold: float,
) -> Dict[str, Dict[str, float]]:
    recommendations: Dict[str, Dict[str, float]] = {}
    for joint_name in JOINT_NAMES:
        values = residuals.get(joint_name, [])
        if not values:
            recommendations[joint_name] = {
                "sample_count": 0,
                "percentile_nm": float("nan"),
                "mean_nm": float("nan"),
                "std_nm": float("nan"),
                "recommended_nm": float(min_threshold),
            }
            continue
        data = np.array(values, dtype=float)
        percentile_value = float(np.percentile(data, percentile))
        mean_value = float(np.mean(data))
        std_value = float(np.std(data))
        recommended = max(min_threshold, percentile_value * margin_scale)
        recommendations[joint_name] = {
            "sample_count": int(len(data)),
            "percentile_nm": percentile_value,
            "mean_nm": mean_value,
            "std_nm": std_value,
            "recommended_nm": recommended,
        }
    return recommendations


def _apply_thresholds(config_path: Path, recommendations: Dict[str, Dict[str, float]]) -> None:
    payload = json.loads(config_path.read_text(encoding="utf-8"))
    payload["external_torque_thresholds"] = [
        recommendations[joint_name]["recommended_nm"] for joint_name in JOINT_NAMES
    ]
    config_path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def main() -> int:
    args = _parse_args()
    input_path = Path(args.input)
    model_path = Path(args.model)
    config_path = Path(args.config)
    if not input_path.exists():
        raise SystemExit(f"Input CSV not found: {input_path}")
    if not model_path.exists():
        raise SystemExit(f"Model JSON not found: {model_path}")

    model = _load_model(model_path)
    residuals = _collect_residuals(
        input_path,
        model,
        max_following_error=float(args.max_following_error),
        max_abs_acceleration=float(args.max_abs_acceleration),
    )
    recommendations = _recommend_thresholds(
        residuals,
        percentile=float(args.percentile),
        margin_scale=float(args.margin_scale),
        min_threshold=float(args.min_threshold),
    )

    for joint_name in JOINT_NAMES:
        item = recommendations[joint_name]
        print(
            f"{joint_name}: samples={item['sample_count']} "
            f"p{args.percentile:g}={item['percentile_nm']:.3f} "
            f"mean={item['mean_nm']:.3f} std={item['std_nm']:.3f} "
            f"recommended={item['recommended_nm']:.3f}"
        )

    if args.apply:
        _apply_thresholds(config_path, recommendations)
        print(f"Applied thresholds to {config_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
