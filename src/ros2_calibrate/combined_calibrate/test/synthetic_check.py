#!/usr/bin/env python3
"""Self check: build shots from a known T_lidar_to_camera and solve them.

    python3 test/synthetic_check.py [work_dir]

Writes a handful of synthetic shot_* folders (PCD + PNG + corners), runs the
C++ solver and checks that the recovered extrinsic matches the ground truth.
Use it after changing the solver to make sure nothing regressed.
"""
from __future__ import annotations

import math
import os
import shutil
import subprocess
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calib_test_util import find_tool  # noqa: E402

# Intrinsics of the dart rig camera after the ROI crop (1536x960 out of the
# full 3072x2048 sensor), as published for the calibration.
FX, FY = 3476.150606, 3474.926803
CX, CY = 1640.995156 - 768.0, 1101.239069 - 544.0
DIST = np.array([0.009749, 0.054858, -0.001340, 0.009647, 0.0])
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
WIDTH, HEIGHT = 1536, 960

BOARD = 0.6  # target side length in metres
R_TRUE, _ = cv2.Rodrigues(np.array([0.06, -0.10, 0.03]))
T_TRUE = np.array([0.12, -0.045, 0.075])
T_LIDAR_TO_CAMERA = np.eye(4)
T_LIDAR_TO_CAMERA[:3, :3] = R_TRUE
T_LIDAR_TO_CAMERA[:3, 3] = T_TRUE


def board_corners_lidar() -> np.ndarray:
    half = BOARD / 2.0
    local = np.array(
        [[-half, -half, 0.0], [half, -half, 0.0], [half, half, 0.0], [-half, half, 0.0]]
    )
    corners = []
    poses = [
        ((0.35, 0.10, 4.2), (0.20, 0.10, 0.05)),
        ((0.05, -0.35, 5.0), (-0.30, 0.20, 0.15)),
        ((0.55, 0.30, 3.4), (0.45, -0.25, -0.10)),
        ((-0.30, 0.05, 6.0), (0.10, 0.40, 0.30)),
        ((0.20, -0.15, 4.6), (-0.15, -0.35, 0.20)),
    ]
    clouds = []
    for shift, rpy in poses:
        rotation, _ = cv2.Rodrigues(np.array(rpy))
        corners.append(local @ rotation.T + np.array(shift))
        rng = np.random.default_rng(7)
        points = np.hstack([rng.uniform(-half, half, (4000, 2)), np.zeros((4000, 1))])
        points += rng.normal(0.0, 0.004, (4000, 3))
        clouds.append(points @ rotation.T + np.array(shift))
    return corners, clouds


def write_shot(directory: str, corners_lidar: np.ndarray, cloud: np.ndarray) -> None:
    os.makedirs(directory, exist_ok=True)
    image_points, _ = cv2.projectPoints(
        corners_lidar, cv2.Rodrigues(R_TRUE)[0], T_TRUE, K, DIST
    )
    image_points = image_points.reshape(-1, 2)

    image = np.full((HEIGHT, WIDTH, 3), 40, np.uint8)
    polygon = np.round(image_points).astype(np.int32)
    cv2.polylines(image, [polygon], True, (200, 200, 200), 3)
    for point in image_points:
        cv2.circle(image, tuple(np.round(point).astype(int)), 6, (0, 0, 255), -1)
    cv2.imwrite(os.path.join(directory, "image.png"), image)

    with open(os.path.join(directory, "cloud.pcd"), "w") as handle:
        handle.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        handle.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        handle.write(f"WIDTH {len(cloud)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        handle.write(f"POINTS {len(cloud)}\nDATA ascii\n")
        np.savetxt(handle, cloud, fmt="%.6f")

    with open(os.path.join(directory, "intrinsics.yaml"), "w") as handle:
        yaml.safe_dump(
            {
                "image_width": WIDTH,
                "image_height": HEIGHT,
                "camera_matrix": [float(v) for v in K.reshape(-1)],
                "distortion": [float(v) for v in DIST],
            },
            handle,
            sort_keys=False,
        )

    with open(os.path.join(directory, "corners_img.txt"), "w") as handle:
        for point in image_points:
            handle.write(f"{point[0]:.4f} {point[1]:.4f}\n")
    with open(os.path.join(directory, "corners_lidar.txt"), "w") as handle:
        for point in corners_lidar:
            handle.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")


def main() -> int:
    work_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/combined_calibrate_synth"
    if os.path.isdir(work_dir):
        shutil.rmtree(work_dir)
    os.makedirs(work_dir)

    np.random.seed(3)
    corners, clouds = board_corners_lidar()

    # noiseless shots first
    for index, (corner, cloud) in enumerate(zip(corners, clouds)):
        write_shot(os.path.join(work_dir, f"shot_{index:03d}"), corner, cloud)

    solver = find_tool("solve_extrinsic")
    if solver is None:
        print("solve_extrinsic not found: build combined_calibrate and source "
              "install/setup.bash, or set COMBINED_CALIBRATE_PREFIX", file=sys.stderr)
        return 1
    result = subprocess.run([solver, work_dir], capture_output=True, text=True)
    print(result.stdout)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        return 1

    with open(os.path.join(work_dir, "extrinsics.yaml")) as handle:
        payload = yaml.safe_load(handle)
    entry = payload["T_lidar_to_camera"]
    estimate = np.asarray(entry["data"], dtype=float).reshape(entry["rows"], entry["cols"])
    error = np.abs(estimate - T_LIDAR_TO_CAMERA).max()
    print(f"max |T_est - T_true| = {error:.3e}")

    with open(os.path.join(work_dir, "intrinsics.yaml_from_ros"), "w") as handle:
        yaml.safe_dump(
            {
                "image_width": 3072,
                "image_height": 2048,
                "camera_name": "camera",
                "camera_matrix": {
                    "rows": 3,
                    "cols": 3,
                    "data": [3476.150606, 0.0, 1640.995156, 0.0, 3474.926803, 1101.239069, 0.0,
                             0.0, 1.0],
                },
                "distortion_model": "plumb_bob",
                "distortion_coefficients": {
                    "rows": 1,
                    "cols": 5,
                    "data": [0.009749, 0.054858, -0.001340, 0.009647, 0.0],
                },
            },
            handle,
            sort_keys=False,
        )
    result = subprocess.run(
        [solver, work_dir, "--intrinsics", os.path.join(work_dir, "intrinsics.yaml_from_ros")],
        capture_output=True,
        text=True,
    )
    print(result.stdout)
    if result.returncode != 0:
        print(result.stderr, file=sys.stderr)
        return 1
    with open(os.path.join(work_dir, "extrinsics.yaml")) as handle:
        payload = yaml.safe_load(handle)
    entry = payload["T_lidar_to_camera"]
    estimate_roi = np.asarray(entry["data"], dtype=float).reshape(entry["rows"], entry["cols"])
    roi_error = np.abs(estimate_roi - T_LIDAR_TO_CAMERA).max()
    print(f"with a full sensor camera_info (ROI correction): max error = {roi_error:.3e}")

    mean_error = float(payload["mean_reprojection_error_px"])
    print(f"mean reprojection error: {mean_error:.4f} px")
    if error > 1e-6 or roi_error > 1e-6 or mean_error > 1e-3:
        print("FAILED")
        return 1
    print("PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
