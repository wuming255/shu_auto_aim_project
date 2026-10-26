#!/usr/bin/env python3
"""Exercise the C++ simulator entry over ROS; Python only supplies test messages."""

import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--launcher", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 40)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    log_path = args.output / "node.log"
    summary_path = args.output / "summary.txt"
    if summary_path.exists():
        summary_path.unlink()
    log = log_path.open("w")
    process = subprocess.Popen([
        "bash", args.launcher, "simulator", "--ros-args",
        "-p", "show_debug:=false", "-p", "classifier_enabled:=false",
        "-p", "control_enabled:=false",
        "-p", "idle_timeout_sec:=8.0", "-p", f"summary_path:={summary_path}",
    ], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    node = rclpy.create_node("simulator_adapter_test")
    images = node.create_publisher(Image, "/image_raw", qos_profile_sensor_data)
    cameras = node.create_publisher(CameraInfo, "/camera_info", qos_profile_sensor_data)
    received = {}
    subscription = node.create_subscription(
        Image, "/srm_simulator/detection/image",
        lambda msg: received.setdefault(msg.header.stamp.sec, msg), qos_profile_sensor_data)

    def wait_for(predicate, seconds=5.0, publish=None):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            check(process.poll() is None,
                  f"C++ simulator entry exited before receiving images:\n{log_path.read_text()}")
            if publish is not None:
                publish()
            rclpy.spin_once(node, timeout_sec=0.05)
        check(predicate(), f"ROS result timed out:\n{log_path.read_text()}")

    try:
        wait_for(lambda: images.get_subscription_count() > 0 and cameras.get_subscription_count() > 0)
        camera = CameraInfo()
        camera.width, camera.height = 320, 240
        camera.distortion_model = "plumb_bob"
        camera.k = [0., 0., 160., 0., 500., 120., 0., 0., 1.]
        camera.d = [0.] * 5
        # Sensor-data delivery is best-effort: wait for observed rejection, not a fixed delay.
        wait_for(lambda: "CameraInfo rejected:" in log_path.read_text(),
                 publish=lambda: cameras.publish(camera))
        malformed = Image()
        malformed.width, malformed.height = 320, 240
        malformed.encoding, malformed.step = "mono8", 320
        malformed.data = bytes(320 * 240)
        wait_for(lambda: "Image rejected:" in log_path.read_text(),
                 publish=lambda: images.publish(malformed))
        check(not received, "malformed image was published as a valid detection")
        camera.k[0] = 500.
        cameras.publish(camera)

        for index, encoding in enumerate(("rgb8", "bgr8", "rgb8", "bgr8")):
            bgr = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.rectangle(bgr, (92, 90), (101, 149), (255, 0, 0), -1)
            cv2.rectangle(bgr, (158, 90), (167, 149), (255, 0, 0), -1)
            cv2.putText(bgr, "2", (112, 144), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (90, 90, 90), 3)
            bgr[10, 10] = (70, 40, 20 + index)
            pixels = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if encoding == "rgb8" else bgr
            padded = np.full((240, 320 * 3 + 8), 231, dtype=np.uint8)
            padded[:, :960] = pixels.reshape(240, 960)
            message = Image()
            message.header.stamp.sec, message.header.stamp.nanosec = 100 + index, 17
            message.header.frame_id = "camera_optical_frame"
            message.width, message.height = 320, 240
            message.encoding, message.step = encoding, 968
            message.data = padded.tobytes()
            images.publish(message)
            wait_for(lambda: 100 + index in received)
            drawing = received[100 + index]
            check(drawing.encoding == "bgr8" and drawing.step == 960,
                  "debug image must have a tightly packed BGR layout")
            check(drawing.header.frame_id == "camera_optical_frame" and
                  drawing.header.stamp.nanosec == 17, "source image header was lost")
            offset = 10 * drawing.step + 10 * 3
            check(list(drawing.data[offset:offset + 3]) == [70, 40, 20 + index],
                  "RGB/BGR channel order or padded-row handling is incorrect")

        command_publishers = node.get_publishers_info_by_topic("/vision_send_data")
        check(not any(info.node_name == "srm_simulator_detector" for info in command_publishers),
              "recognition entry must not publish gimbal commands")
        process.send_signal(signal.SIGINT)
        check(process.wait(timeout=5) == 0, f"node failed:\n{log_path.read_text()}")
        values = dict(line.split("=", 1) for line in summary_path.read_text().splitlines())
        check(int(values["processed_frames"]) == 4, "valid image count is incorrect")
        check(int(values["frames_with_armors"]) == 4 and int(values["armor_count"]) == 4,
              "received images did not pass through the existing C++ armor detector")
        check(int(values["rejected_images"]) >= 1, "malformed input was not accounted for")
        check(int(values["rejected_camera_info"]) >= 1 and values["camera_info_ready"] == "1",
              "camera-info validation/recovery failed")
        print("C++ simulator ROS smoke passed: RGB/BGR, row padding, headers, armor detection, invalid input, shutdown")
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == "__main__":
    main()
