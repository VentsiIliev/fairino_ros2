#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Tuple

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
MODEL_VERSION = 2


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
    parser.add_argument(
        "--update-existing",
        action="store_true",
        help="Incrementally update an existing model by consuming only new appended rows when possible",
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


def _load_rows(
    csv_path: Path,
    max_following_error: float,
    max_abs_acceleration: float,
    skip_data_rows: int = 0,
) -> Tuple[Dict[str, List[List[float]]], int]:
    rows_by_joint: Dict[str, List[List[float]]] = defaultdict(list)
    data_row_count = 0
    with csv_path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            data_row_count += 1
            if data_row_count <= skip_data_rows:
                continue
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
    return rows_by_joint, data_row_count


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


def _joint_stats_from_samples(samples: np.ndarray) -> Dict[str, object]:
    design = _design_matrix(samples)
    tau = samples[:, 3]
    xtx = design.T @ design
    xty = design.T @ tau
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
        "normal_equations": {
            "xtx": xtx.tolist(),
            "xty": xty.tolist(),
        },
    }


def _load_existing_model(model_path: Path) -> dict | None:
    if not model_path.exists():
        return None
    payload = json.loads(model_path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise SystemExit(f"Invalid model JSON: {model_path}")
    return payload


def _normalize_model_path(path: Path) -> str:
    return str(path.expanduser().resolve())


def _model_supports_incremental(payload: dict | None) -> bool:
    if not isinstance(payload, dict):
        return False
    joints_payload = payload.get("joints", {})
    if not isinstance(joints_payload, dict):
        return False
    for joint_payload in joints_payload.values():
        if not isinstance(joint_payload, dict) or joint_payload.get("skipped"):
            continue
        stats_payload = joint_payload.get("normal_equations", {})
        xtx = stats_payload.get("xtx")
        xty = stats_payload.get("xty")
        if isinstance(xtx, list) and isinstance(xty, list):
            return True
    return False


def _update_joint_from_stats(existing_joint: dict, samples: np.ndarray) -> Dict[str, object]:
    stats_payload = existing_joint.get("normal_equations", {})
    xtx = np.array(stats_payload.get("xtx", []), dtype=float)
    xty = np.array(stats_payload.get("xty", []), dtype=float)
    if xtx.shape != (len(FEATURE_NAMES), len(FEATURE_NAMES)) or xty.shape != (len(FEATURE_NAMES),):
        raise SystemExit("Existing model is missing usable normal-equation state for incremental update")

    design = _design_matrix(samples)
    tau = samples[:, 3]
    xtx += design.T @ design
    xty += design.T @ tau
    coeffs = np.linalg.solve(xtx, xty)
    prediction = design @ coeffs
    residual = tau - prediction
    rmse = float(np.sqrt(np.mean(np.square(residual))))
    mae = float(np.mean(np.abs(residual)))
    max_abs_error = float(np.max(np.abs(residual)))
    previous_count = int(existing_joint.get("sample_count", 0))
    return {
        "feature_names": FEATURE_NAMES,
        "coefficients": [float(value) for value in coeffs],
        "rmse_nm": rmse,
        "mae_nm": mae,
        "max_abs_error_nm": max_abs_error,
        "sample_count": previous_count + int(len(samples)),
        "normal_equations": {
            "xtx": xtx.tolist(),
            "xty": xty.tolist(),
        },
    }


def main() -> int:
    args = _parse_args()
    input_path = Path(args.input)
    output_path = Path(args.output)
    if not input_path.exists():
        raise SystemExit(f"Input CSV not found: {input_path}")

    existing_model = _load_existing_model(output_path) if args.update_existing else None
    normalized_input_path = _normalize_model_path(input_path)
    skip_data_rows = 0
    base_output: Dict[str, object] | None = None
    incremental_enabled = args.update_existing and _model_supports_incremental(existing_model)
    if existing_model is not None and incremental_enabled:
        existing_input_state = existing_model.get("input_state", {})
        if isinstance(existing_input_state, dict):
            existing_input_path = str(existing_input_state.get("path", ""))
            existing_row_count = int(existing_input_state.get("data_row_count", 0))
            if existing_input_path == normalized_input_path:
                skip_data_rows = max(existing_row_count, 0)
        base_output = existing_model

    rows_by_joint, data_row_count = _load_rows(
        input_path,
        max_following_error=float(args.max_following_error),
        max_abs_acceleration=float(args.max_abs_acceleration),
        skip_data_rows=skip_data_rows,
    )

    output: Dict[str, object] = {
        "model_version": MODEL_VERSION,
        "input_csv": str(input_path),
        "feature_names": FEATURE_NAMES,
        "joints": {},
        "input_state": {
            "path": normalized_input_path,
            "data_row_count": data_row_count,
        },
    }
    if base_output is not None:
        output.update({key: value for key, value in base_output.items() if key not in output})
        output["model_version"] = MODEL_VERSION
        output["input_csv"] = str(input_path)
        output["feature_names"] = FEATURE_NAMES
        output["input_state"] = {
            "path": normalized_input_path,
            "data_row_count": data_row_count,
        }
        output["joints"] = {}

    for joint_name in JOINT_NAMES:
        samples_list = rows_by_joint.get(joint_name, [])
        existing_joint = None
        if base_output is not None:
            existing_joint = (base_output.get("joints", {}) or {}).get(joint_name)

        if incremental_enabled and existing_joint and skip_data_rows > 0 and not samples_list:
            output["joints"][joint_name] = existing_joint
            continue

        if len(samples_list) < int(args.min_samples):
            if existing_joint and incremental_enabled and skip_data_rows > 0:
                output["joints"][joint_name] = existing_joint
                continue
            output["joints"][joint_name] = {
                "sample_count": len(samples_list),
                "skipped": True,
                "reason": f"need at least {args.min_samples} samples",
            }
            continue

        samples = np.array(samples_list, dtype=float)
        if existing_joint and incremental_enabled:
            output["joints"][joint_name] = _update_joint_from_stats(existing_joint, samples)
        else:
            output["joints"][joint_name] = _joint_stats_from_samples(samples)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2, sort_keys=True), encoding="utf-8")

    print(f"Wrote torque model to {output_path}")
    if args.update_existing:
        consumed_rows = max(data_row_count - skip_data_rows, 0)
        mode = "incremental" if incremental_enabled else "full_rebuild_for_legacy_model"
        print(
            f"Update mode={mode}: previous_rows={skip_data_rows} "
            f"new_rows={consumed_rows} total_rows={data_row_count}"
        )
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
