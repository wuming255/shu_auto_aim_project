#!/usr/bin/env python3
"""Catch missing/wrong ROS control, mount-angle conversion, fire edges and stale cancellation."""

import argparse
from array import array
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from rm_interfaces.msg import VisionReceiveData, VisionSendData
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage


def rotation(yaw, pitch_up):
    y, p = math.radians(yaw), math.radians(pitch_up)
    return np.array([[math.cos(y), -math.sin(y), 0.],
                     [math.sin(y), math.cos(y), 0.], [0., 0., 1.]]) @ np.array([
                         [math.cos(p), 0., -math.sin(p)], [0., 1., 0.],
                         [math.sin(p), 0., math.cos(p)]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--launcher", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 40)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    log_path = args.output / "node.log"
    summary = args.output / "summary.txt"
    log = log_path.open("w")
    process = subprocess.Popen([
        "bash", args.launcher, "simulator", "--ros-args", "-p", "show_debug:=false",
        "-p", "classifier_enabled:=false", "-p", "allow_visual_fire:=true",
        "-p", "idle_timeout_sec:=8.0", "-p", f"summary_path:={summary}",
    ], stdout=log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    node = rclpy.create_node("simulator_control_fixture")
    image_pub = node.create_publisher(Image, "/image_raw", qos_profile_sensor_data)
    camera_pub = node.create_publisher(CameraInfo, "/camera_info", qos_profile_sensor_data)
    feedback_pub = node.create_publisher(VisionReceiveData, "/vision_receive_data", qos_profile_sensor_data)
    tf_pub = node.create_publisher(TFMessage, "/tf", 10)
    commands = []
    node.create_subscription(VisionSendData, "/vision_send_data",
                             lambda msg: commands.append(msg), qos_profile_sensor_data)
    camera = CameraInfo()
    camera.width, camera.height = 640, 480
    camera.distortion_model = "plumb_bob"
    camera.k = [600., 0., 320., 0., 600., 240., 0., 0., 1.]
    camera.d = [0.] * 5
    optical_to_flu = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    mount = rotation(0., 25.)
    camera_local, muzzle_local = np.array([.05, .003, .12]), np.array([0., 0., .04])
    target = np.array([2., .2, .1])
    # Fixed world board: right=-Y, up=+Z, normal=-X. Only message fixtures use this truth.
    board = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
    corners = np.array([[-.0629, .029805, 0.], [.0629, .029805, 0.],
                        [.0629, -.029805, 0.], [-.0629, -.029805, 0.]])
    yaw, pitch = 20., 20.

    def check(condition, message):
        if not condition:
            raise AssertionError(message + "\n" + log_path.read_text())

    def spin(seconds=.04):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            check(process.poll() is None, "C++ control entry exited unexpectedly")
            rclpy.spin_once(node, timeout_sec=.01)

    def publish(visible=True, feedback=True):
        nonlocal yaw, pitch
        stamp = node.get_clock().now().to_msg()
        raw = rotation(yaw, -pitch)
        barrel = raw @ mount
        if feedback:
            state = VisionReceiveData()
            state.header.stamp = stamp
            state.header.frame_id = "gimbal_link"
            state.self_color, state.bullet_speed = 1, 25
            state.yaw, state.pitch = yaw, pitch
            feedback_pub.publish(state)
            tf = TFMessage()
            for parent, child, translation, matrix in (
                    ("map", "odom", np.zeros(3), np.eye(3)),
                    ("odom", "gimbal_link", np.zeros(3), barrel),
                    ("gimbal_link", "camera_link", camera_local, np.eye(3)),
                    ("gimbal_link", "muzzle", muzzle_local, np.eye(3))):
                item = TransformStamped()
                item.header.stamp, item.header.frame_id, item.child_frame_id = stamp, parent, child
                item.transform.translation.x, item.transform.translation.y, item.transform.translation.z = map(float, translation)
                # Positive trace in this bounded fixture, independent of C++ quaternion conversion.
                w = math.sqrt(1. + np.trace(matrix)) / 2.
                item.transform.rotation.w = w
                item.transform.rotation.x = (matrix[2, 1] - matrix[1, 2]) / (4. * w)
                item.transform.rotation.y = (matrix[0, 2] - matrix[2, 0]) / (4. * w)
                item.transform.rotation.z = (matrix[1, 0] - matrix[0, 1]) / (4. * w)
                tf.transforms.append(item)
            tf_pub.publish(tf)
        camera.header.stamp = stamp
        camera_pub.publish(camera)
        image = np.zeros((480, 640, 3), np.uint8)
        if visible:
            world_points = corners @ board.T + target
            camera_position = raw @ (camera_local - muzzle_local)
            points = (world_points - camera_position) @ (barrel @ optical_to_flu)
            pixels = np.rint(points[:, :2] / points[:, 2:] * 600. + [320., 240.]).astype(int)
            cv2.line(image, tuple(pixels[0]), tuple(pixels[3]), (255, 0, 0), 2)
            cv2.line(image, tuple(pixels[1]), tuple(pixels[2]), (255, 0, 0), 2)
            center = pixels.mean(axis=0).astype(int)
            cv2.putText(image, "2", (int(center[0]) - 8, int(center[1]) + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, .8, (100, 100, 100), 2)
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, "camera_optical_frame"
        msg.width, msg.height, msg.encoding, msg.step = 640, 480, "bgr8", 1920
        # array('B') takes the generated ROS setter's buffer path; assigning bytes
        # validates every megapixel in Python and makes the fixture itself stale.
        msg.data = array("B", image.tobytes())
        image_pub.publish(msg)

    try:
        deadline = time.monotonic() + 5.
        while not image_pub.get_subscription_count() and time.monotonic() < deadline:
            spin()
        # The old recognition-only implementation reaches this assertion and fails here.
        check(feedback_pub.get_subscription_count() > 0,
              "missing simulator feedback interface /vision_receive_data")
        deadline = time.monotonic() + 7.
        def fire_edges():
            return sum(c.target_state == 2 and (i == 0 or commands[i - 1].target_state != 2)
                       for i, c in enumerate(commands))

        while time.monotonic() < deadline and fire_edges() < 2:
            publish()
            spin(.05)
            if commands and commands[-1].target_state:
                yaw, pitch = float(commands[-1].yaw), float(commands[-1].pitch)
        valid = [c for c in commands if c.target_state]
        check(bool(valid), "PnP/tracking/ballistics did not produce a gimbal command")
        final = valid[-1]
        # Analytic static low trajectory: yaw=atan(.2/2)=5.71deg, gimbal pitch about 21.2deg.
        check(abs(final.yaw - 5.71) < 1.5 and abs(final.pitch - 21.2) < 2.,
              f"wrong world-angle/mount conversion: yaw={final.yaw}, pitch={final.pitch}")
        check(1.5 < final.target_distance < 2.5, "wrong metric feature geometry or distance mapping")
        check(fire_edges() >= 2, "fire requests were not released and re-armed as rising edges")
        # Keep valid images while cutting feedback: the command watchdog must cancel.
        for _ in range(10):
            publish(feedback=False)
            spin(.05)
        check(commands[-1].target_state == 0 and commands[-1].target_distance == -1.,
              "stale feedback did not cancel the simulator target")
        for _ in range(8):
            publish()
            spin(.05)
        check(commands[-1].target_state != 0, "control did not recover after feedback resumed")
        for _ in range(8):
            publish(visible=False)
            spin(.05)
        check(commands[-1].target_state == 0, "target loss kept an active follow/fire request")
        process.send_signal(signal.SIGINT)
        check(process.wait(timeout=5) == 0, "control node failed to stop")
        values = dict(line.split("=", 1) for line in summary.read_text().splitlines())
        check(int(values["aim_command_count"]) > 0 and int(values["fire_pulse_count"]) >= 2,
              "control summary did not record actual outputs")
        print("C++ simulator control smoke passed: geometry, feedback, ballistics, fire edges, stale/lost cancellation, recovery")
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
        log.close()


if __name__ == "__main__":
    main()
