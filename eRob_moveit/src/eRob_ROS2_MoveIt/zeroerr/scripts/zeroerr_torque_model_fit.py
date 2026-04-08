#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, List

import numpy as np


DEFAULT_INPUT = Path(__file__).resolve().parents[1] / "data" / "torque_sensor_log.csv"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "config" / "torque_sensor_model.json"
JOINT_NAMES = [f"Joint_{index}" for index in range(1, 7)]
FEATURE_NAMES = [
    "bias",
    "sin_q",
    "cos_q",
    "dq",
    "sign_dq",
    "ddq",
]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fit a simple per-joint nominal torque model from ZeroErr torque sensor CSV logs."
    )
    parser.add_argument("--input", default=str(DEFAULT_INPUT), help="Input CSV log path")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Output JSON model path")
    parser.add_argument(
        "--min-samples",
        type=int,
        default=100,
        help="Minimum samples required per joint before fitting",
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


def _load_rows(csv_path: Path, max_following_error: float, max_abs_acceleration: float) -> Dict[str, List[List[float]]]:
    rows_by_joint: Dict[str, List[List[float]]] = defaultdict(list)
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            joint = row.get("joint", "")
            if joint not in JOINT_NAMES:
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

            rows_by_joint[joint].append([q, dq, ddq, tau])
    return rows_by_joint


def _design_matrix(samples: np.ndarray) -> np.ndarray:
    q = samples[:, 0]
    dq = samples[:, 1]
    ddq = samples[:, 2]
    return np.column_stack(
        [
            np.ones(len(samples)),
            np.sin(q),
            np.cos(q),
            dq,
            np.sign(dq),
            ddq,
        ]
    )


def _fit_joint(samples: np.ndarray) -> Dict[str, object]:
    design = _design_matrix(samples)
    tau = samples[:, 3]
    coeffs, *_ = np.linalg.lstsq(design, tau, rcond=None)
    prediction = design @ coeffs
    residual = tau - prediction
    rmse = float(np.sqrt(np.mean(np.square(residual))))
    mae = float(np.mean(np.abs(residual)))
    max_abs_error = float(np.max(np.abs(residual)))
    return {
        "feature_names": FEATURE_NAMES,
        "coefficients": [float(value) for value in coeffs],
        "rmse_nm": rmse,
        "mae_nm": mae,
        "max_abs_error_nm": max_abs_error,
        "sample_count": int(len(samples)),
    }


def main() -> int:
    args = _parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)
    if not input_path.exists():
        raise SystemExit(f"Input CSV not found: {input_path}")

    rows_by_joint = _load_rows(
        input_path,
        max_following_error=float(args.max_following_error),
        max_abs_acceleration=float(args.max_abs_acceleration),
    )

    output: Dict[str, object] = {
        "input_csv": str(input_path),
        "feature_names": FEATURE_NAMES,
        "joints": {},
    }

    for joint_name in JOINT_NAMES:
        samples_list = rows_by_joint.get(joint_name, [])
        if len(samples_list) < int(args.min_samples):
            output["joints"][joint_name] = {
                "sample_count": len(samples_list),
                "skipped": True,
                "reason": f"need at least {args.min_samples} samples",
            }
            continue

        samples = np.array(samples_list, dtype=float)
        output["joints"][joint_name] = _fit_joint(samples)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2, sort_keys=True), encoding="utf-8")

    print(f"Wrote torque model to {output_path}")
    for joint_name in JOINT_NAMES:
        joint_result = output["joints"][joint_name]
        sample_count = joint_result.get("sample_count", 0)
        if joint_result.get("skipped"):
            print(f"{joint_name}: skipped ({sample_count} samples)")
            continue
        print(
            f"{joint_name}: samples={sample_count} "
            f"rmse={joint_result['rmse_nm']:.3f} "
            f"mae={joint_result['mae_nm']:.3f} "
            f"max_abs={joint_result['max_abs_error_nm']:.3f}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
