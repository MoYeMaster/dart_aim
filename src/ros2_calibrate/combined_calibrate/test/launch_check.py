#!/usr/bin/env python3
"""Check the single launch entry point without hardware.

Starts a synthetic scene, then runs
    ros2 launch combined_calibrate calibrate.launch.py stage:=verify start_drivers:=false ...
and verifies that the launched node subscribes to the driver topics and
publishes the projection overlay.

    python3 test/launch_check.py <shots_dir>      # shots_dir needs extrinsics.yaml
"""
from __future__ import annotations

import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ros_pipeline_check import Scene  # noqa: E402  (synthetic publisher)


class Counter(rclpy.node.Node):
    def __init__(self):
        super().__init__("launch_check_counter")
        self.count = 0
        self.create_subscription(Image, "/calibrate/overlay", self.on_image, 2)

    def on_image(self, _msg):
        self.count += 1


def main() -> int:
    shots_dir = sys.argv[1] if len(sys.argv) > 1 else "/tmp/cb_pipe/shots"
    rclpy.init()
    scene = Scene()
    counter = Counter()
    executor = SingleThreadedExecutor()
    executor.add_node(scene)
    executor.add_node(counter)
    threading.Thread(target=executor.spin, daemon=True).start()

    process = subprocess.Popen(
        [
            "ros2", "launch", "combined_calibrate", "calibrate.launch.py",
            "stage:=verify", "start_drivers:=false", "use_rviz:=false",
            "shots_dir:=" + shots_dir,
            "interactive:=false", "accumulate_frames:=3", "publish_rate_hz:=5.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    deadline = time.time() + 25.0
    while time.time() < deadline and counter.count == 0:
        time.sleep(0.2)
    time.sleep(0.5)
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        output = process.communicate(timeout=15.0)[0]
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        output = process.communicate()[0]

    print(output)
    executor.shutdown()
    rclpy.try_shutdown()
    if counter.count == 0:
        print("FAILED: the launched node published no overlay")
        return 1
    print(f"PASSED: {counter.count} overlay frames from the launch file")
    return 0


if __name__ == "__main__":
    status = main()
    sys.stdout.flush()
    os._exit(status)
