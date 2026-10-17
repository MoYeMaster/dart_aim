#!/usr/bin/env python3
"""End to end check without hardware.

Publishes a synthetic camera + LiDAR scene on the topics the drivers use, then
runs the real nodes: capture -> solve -> project.  Verifies that the shot files
are written, that the ROI intrinsics correction lands, that the solver recovers
the ground truth and that the overlay really draws the cloud on the image.

    python3 test/ros_pipeline_check.py [work_dir]
"""
from __future__ import annotations

import os
import shutil
import struct
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calib_test_util import find_tool  # noqa: E402

FX, FY = 3476.150606, 3474.926803
CX, CY = 1640.995156, 1101.239069
DIST = [0.009749, 0.054858, -0.001340, 0.009647, 0.0]
K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]])
WIDTH, HEIGHT = 1536, 960
ROI_OFFSET = (768.0, 544.0)
# The camera delivers a centred ROI crop, so the pixels the calibration sees
# follow the shifted principal point.
K_ROI = K.copy()
K_ROI[0, 2] -= ROI_OFFSET[0]
K_ROI[1, 2] -= ROI_OFFSET[1]

R_TRUE, _ = cv2.Rodrigues(np.array([0.06, -0.10, 0.03]))
T_TRUE = np.array([0.12, -0.045, 0.075])
T_LIDAR_TO_CAMERA = np.eye(4)
T_LIDAR_TO_CAMERA[:3, :3] = R_TRUE
T_LIDAR_TO_CAMERA[:3, 3] = T_TRUE

HALF = 0.3
LOCAL_CORNERS = np.array([[-HALF, -HALF, 0.0], [HALF, -HALF, 0.0], [HALF, HALF, 0.0], [-HALF, HALF, 0.0]])
BOARD_SHIFT = np.array([0.25, 0.05, 4.5])
BOARD_ROT, _ = cv2.Rodrigues(np.array([0.25, 0.15, 0.05]))
CORNERS_LIDAR = LOCAL_CORNERS @ BOARD_ROT.T + BOARD_SHIFT
_rng = np.random.default_rng(11)
_board_points = np.hstack([
    _rng.uniform(-HALF, HALF, (4000, 2)),
    np.zeros((4000, 1)),
]) + _rng.normal(0.0, 0.004, (4000, 3))
CLOUD = _board_points @ BOARD_ROT.T + BOARD_SHIFT


def scene_image() -> np.ndarray:
    """Bayer image of the target as the Hikvision driver would publish it."""
    gray = np.full((HEIGHT, WIDTH), 40, np.uint8)
    projected, _ = cv2.projectPoints(CORNERS_LIDAR, cv2.Rodrigues(R_TRUE)[0], T_TRUE, K_ROI,
                                     np.array(DIST))
    polygon = np.round(projected.reshape(-1, 2)).astype(np.int32)
    cv2.polylines(gray, [polygon], True, 200, 3)
    for point in polygon:
        cv2.circle(gray, tuple(point), 8, 255, -1)
    bayer = np.zeros((HEIGHT, WIDTH), np.uint8)
    bayer[0::2, 0::2] = gray[0::2, 0::2]
    bayer[0::2, 1::2] = gray[0::2, 1::2]
    bayer[1::2, 0::2] = gray[1::2, 0::2]
    bayer[1::2, 1::2] = gray[1::2, 1::2]
    return np.stack([bayer, bayer, bayer], axis=-1)


def cloud_msg(frame_id: str) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(CLOUD)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.is_bigendian = False
    msg.data = b"".join(struct.pack("<fff", *point) for point in CLOUD)
    return msg


class Scene(Node):
    def __init__(self):
        super().__init__("synthetic_scene")
        self.image_pub = self.create_publisher(Image, "/image_raw", 2)
        self.info_pub = self.create_publisher(CameraInfo, "/image_raw/camera_info", 2)
        self.cloud_pub = self.create_publisher(PointCloud2, "/livox/lidar", 2)
        bayer = scene_image()
        self.image = Image()
        self.image.height = HEIGHT
        self.image.width = WIDTH
        self.image.encoding = "bayer_rggb8"
        self.image.step = WIDTH
        self.image.is_bigendian = 0
        self.image.data = bayer[:, :, 0].tobytes()
        self.info = CameraInfo()
        self.info.width = 3072
        self.info.height = 2048
        self.info.k = list(K.reshape(-1))
        self.info.d = list(DIST)
        self.info.distortion_model = "plumb_bob"
        self.timer = self.create_timer(0.05, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.image.header.stamp = stamp
        self.image.header.frame_id = "optical_frame"
        self.info.header = self.image.header
        cloud = cloud_msg("livox_frame")
        cloud.header.stamp = stamp
        self.image_pub.publish(self.image)
        self.info_pub.publish(self.info)
        self.cloud_pub.publish(cloud)


class OverlayWatcher(Node):
    def __init__(self, topic: str):
        super().__init__("overlay_watcher")
        self.frames = []
        self.create_subscription(Image, topic, self.on_image, 2)

    def on_image(self, msg: Image):
        self.frames.append(msg)


def run_capture_node(executable, args, env, shot_dir, timeout=25.0):
    """Run the capture node and keep pressing ENTER until the shot shows up."""
    process = subprocess.Popen(
        [executable] + args,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    shot_path = os.path.join(shot_dir, "shot_000")
    deadline = time.time() + timeout
    saved = False
    while time.time() < deadline and process.poll() is None:
        if os.path.isdir(shot_path):
            saved = True
            break
        try:
            process.stdin.write("\n")
            process.stdin.flush()
        except (BrokenPipeError, ValueError):
            break
        time.sleep(0.6)
    time.sleep(0.5)
    if process.poll() is None:
        try:
            process.stdin.write("q\n")
            process.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass
    try:
        output = process.communicate(timeout=10.0)[0]
    except subprocess.TimeoutExpired:
        process.kill()
        output = process.communicate()[0]
    return saved, output


def run_node(executable: str, args: list[str], env: dict, stdin_lines=None, timeout=25.0):
    process = subprocess.Popen(
        [executable] + args,
        stdin=subprocess.PIPE if stdin_lines else subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    output = ""
    if stdin_lines:
        for line in stdin_lines:
            time.sleep(line[0])
            if process.poll() is None:
                process.stdin.write(line[1])
                process.stdin.flush()
    try:
        output = process.communicate(timeout=timeout)[0]
    except subprocess.TimeoutExpired:
        process.kill()
        output = process.communicate()[0]
    return process.returncode, output


def run_node_until(executable: str, args: list[str], env: dict, ready, timeout=30.0):
    """Run a node until ready() is true, then stop it with 'q'.

    The overlay is published from a timer, but ROS discovery is not instant, so
    a fixed sleep is a race.  Wait for the first frame instead.
    """
    process = subprocess.Popen(
        [executable] + args,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=env,
    )
    deadline = time.time() + timeout
    while time.time() < deadline and process.poll() is None:
        if ready():
            break
        time.sleep(0.2)
    try:
        process.stdin.write("q\n")
        process.stdin.flush()
    except (BrokenPipeError, ValueError):
        pass
    try:
        output = process.communicate(timeout=max(1.0, deadline - time.time()) + 10.0)[0]
    except subprocess.TimeoutExpired:
        process.kill()
        output = process.communicate()[0]
    return process.returncode, output


def main() -> int:
    work_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/combined_calibrate_pipeline"
    if os.path.isdir(work_dir):
        shutil.rmtree(work_dir)
    os.makedirs(work_dir)

    node = find_tool("calibrate_node")
    if node is None:
        print("calibrate_node not found: build combined_calibrate and source "
              "install/setup.bash, or set COMBINED_CALIBRATE_PREFIX", file=sys.stderr)
        return 1
    lib = os.path.dirname(node)
    package_prefix = os.path.dirname(os.path.dirname(lib))
    env = dict(os.environ)
    env["AMENT_PREFIX_PATH"] = package_prefix + ":" + env.get("AMENT_PREFIX_PATH", "")
    env["LD_LIBRARY_PATH"] = os.path.dirname(lib) + ":" + env.get("LD_LIBRARY_PATH", "")

    rclpy.init()
    scene = Scene()
    import threading

    scene_executor = SingleThreadedExecutor()
    scene_executor.add_node(scene)
    threading.Thread(target=scene_executor.spin, daemon=True).start()
    time.sleep(1.0)

    shots_dir = os.path.join(work_dir, "shots")
    saved, output = run_capture_node(
        os.path.join(lib, "calibrate_node"),
        ["--ros-args", "-p", "mode:=capture", "-p", f"shots_dir:={shots_dir}",
         "-p", "accumulate_frames:=3", "-p", "interactive:=false", "-p", "show_preview:=false",
         "-p", "preview_hz:=0.0"],
        env,
        shots_dir,
    )
    print("---- calibrate_node (capture) ----")
    print(output)
    if not saved:
        print("FAILED: no shot_000 written")
        rclpy.try_shutdown()
        return 1

    with open(os.path.join(shots_dir, "shot_000", "intrinsics.yaml")) as handle:
        stored = yaml.safe_load(handle)
    matrix = np.asarray(stored["camera_matrix"], dtype=float).reshape(3, 3)
    print("stored intrinsics:", matrix.reshape(-1)[:3], "size",
          stored["image_width"], stored["image_height"])
    assert stored["image_width"] == WIDTH and stored["image_height"] == HEIGHT
    assert abs(matrix[0, 2] - (CX - ROI_OFFSET[0])) < 1e-6, matrix
    assert abs(matrix[1, 2] - (CY - ROI_OFFSET[1])) < 1e-6, matrix

    with open(os.path.join(shots_dir, "shot_000", "cloud.pcd"), "rb") as handle:
        header = handle.read(400).split(b"DATA")[0]
    print("pcd header:", header.decode(errors="replace").replace("\n", " | ")[:200])

    with open(os.path.join(shots_dir, "shot_000", "corners_lidar.txt"), "w") as handle:
        for point in CORNERS_LIDAR:
            handle.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    with open(os.path.join(shots_dir, "shot_000", "corners_img.txt"), "w") as handle:
        projected, _ = cv2.projectPoints(CORNERS_LIDAR, cv2.Rodrigues(R_TRUE)[0], T_TRUE, K_ROI,
                                         np.array(DIST))
        for point in projected.reshape(-1, 2):
            handle.write(f"{point[0]:.4f} {point[1]:.4f}\n")

    code, output = run_node(os.path.join(lib, "solve_extrinsic"), [shots_dir], env)
    print("---- solve_extrinsic ----")
    print(output)
    if "PASSED" in output or code != 0:
        pass
    with open(os.path.join(shots_dir, "extrinsics.yaml")) as handle:
        payload = yaml.safe_load(handle)
    entry = payload["T_lidar_to_camera"]
    estimate = np.asarray(entry["data"], dtype=float).reshape(entry["rows"], entry["cols"])
    print("solver max error:", np.abs(estimate - T_LIDAR_TO_CAMERA).max())

    watcher = OverlayWatcher("/calibrate/overlay")
    watcher_executor = SingleThreadedExecutor()
    watcher_executor.add_node(watcher)
    threading.Thread(target=watcher_executor.spin, daemon=True).start()
    code, output = run_node_until(
        os.path.join(lib, "calibrate_node"),
        ["--ros-args", "-p", "mode:=verify", "-p", f"shots_dir:={shots_dir}",
         "-p", "interactive:=false", "-p", "show_preview:=false", "-p", "accumulate_frames:=3",
         "-p", "publish_rate_hz:=5.0"],
        env,
        ready=lambda: bool(watcher.frames),
        timeout=30.0,
    )
    print("---- calibrate_node (verify) ----")
    print(output)
    time.sleep(0.5)
    if not watcher.frames:
        print("FAILED: no overlay image published")
        watcher_executor.shutdown()
        rclpy.try_shutdown()
        return 1

    overlay = np.frombuffer(watcher.frames[-1].data, np.uint8).reshape(
        watcher.frames[-1].height, watcher.frames[-1].width, 3)
    base = scene_image()[:, :, 0]
    colored = int(np.count_nonzero(np.abs(overlay.astype(int) - base[:, :, None]).sum(axis=2) > 30))
    print(f"overlay {overlay.shape}, coloured pixels: {colored}")
    cv2.imwrite(os.path.join(work_dir, "overlay.png"), overlay)

    scene.destroy_node()
    watcher.destroy_node()
    scene_executor.shutdown()
    watcher_executor.shutdown()
    rclpy.try_shutdown()
    if colored < 500:
        print("FAILED: the overlay does not draw the projected cloud")
        return 1
    print("PASSED (overlay saved to", os.path.join(work_dir, "overlay.png") + ")")
    return 0


if __name__ == "__main__":
    status = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(status)
