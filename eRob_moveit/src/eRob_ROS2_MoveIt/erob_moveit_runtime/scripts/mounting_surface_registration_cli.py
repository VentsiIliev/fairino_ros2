#!/usr/bin/env python3
"""Manually solve a mounting-surface registration from three point pairs."""

import argparse
import json
import sys

import numpy as np

from calibration.mounting_surface_registration import (
    solve_mounting_surface_frame,
    solve_mounting_surface_registration,
)


def _parse_points(value: str) -> np.ndarray:
    """Parse ``x,y,z;x,y,z;x,y,z`` from the command line."""
    try:
        points = [[float(axis) for axis in point.split(",")] for point in value.split(";")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "points must be written as x,y,z;x,y,z;x,y,z"
        ) from exc
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise argparse.ArgumentTypeError(
            "points must be written as x,y,z;x,y,z;x,y,z"
        )
    return array


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Solve p_base_link = R * p_mounting + t from corresponding points. "
            "Use three or more points."
        )
    )
    parser.add_argument(
        "--mounting-points",
        "--cad-points",
        type=_parse_points,
        help="CAD/mounting-frame points: x,y,z;x,y,z;x,y,z",
    )
    parser.add_argument(
        "--measured-points",
        type=_parse_points,
        help="Measured base_link points in the same order",
    )
    parser.add_argument(
        "--units",
        choices=("m", "mm"),
        default="m",
        help="Units of both input point lists (default: m)",
    )
    parser.add_argument(
        "--measured-origin",
        type=_parse_points,
        help="Frame workflow: measured center point x,y,z",
    )
    parser.add_argument(
        "--measured-x-point",
        type=_parse_points,
        help="Frame workflow: measured point in mounting +X direction",
    )
    parser.add_argument(
        "--measured-y-point",
        type=_parse_points,
        help="Frame workflow: measured point in mounting +Y direction",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print a machine-readable result",
    )
    args = parser.parse_args()

    frame_args = (
        args.measured_origin,
        args.measured_x_point,
        args.measured_y_point,
    )
    point_args = (args.mounting_points, args.measured_points)
    if any(value is not None for value in frame_args):
        if not all(value is not None for value in frame_args):
            parser.error(
                "--measured-origin, --measured-x-point, and --measured-y-point "
                "must be supplied together"
            )
        if any(value is not None for value in point_args):
            parser.error("Use either frame points or corresponding point pairs")
    elif not all(value is not None for value in point_args):
        parser.error(
            "Provide either --mounting-points and --measured-points, or the "
            "three measured frame points"
        )

    scale = 0.001 if args.units == "mm" else 1.0
    try:
        if all(value is not None for value in frame_args):
            result = solve_mounting_surface_frame(
                args.measured_origin[0] * scale,
                args.measured_x_point[0] * scale,
                args.measured_y_point[0] * scale,
            )
        else:
            result = solve_mounting_surface_registration(
                args.mounting_points * scale,
                args.measured_points * scale,
            )
    except ValueError as exc:
        parser.error(str(exc))

    payload = {
        "frame_mapping": "mounting/CAD -> base_link",
        "transform": result.transform.tolist(),
        "rotation": result.rotation.tolist(),
        "translation_m": result.translation_m.tolist(),
        "translation_mm": result.translation_mm.tolist(),
        "residuals_m": result.residuals_m.tolist(),
        "residuals_mm": (result.residuals_m * 1000.0).tolist(),
        "rms_error_mm": result.rms_error_m * 1000.0,
        "max_error_mm": result.max_error_m * 1000.0,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
        return 0

    np.set_printoptions(precision=9, suppress=True)
    print("Transform: mounting/CAD -> base_link")
    print("T_base_mounting =")
    print(result.transform)
    print(f"translation [mm] = {result.translation_mm}")
    print(f"point residuals [mm] = {result.residuals_m * 1000.0}")
    print(f"RMS residual [mm] = {result.rms_error_m * 1000.0:.6f}")
    print(f"max residual [mm] = {result.max_error_m * 1000.0:.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
