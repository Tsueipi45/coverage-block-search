#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import os
import sys
import threading

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from coverage_common import read_color_list


class BlockDetector:
    def __init__(self):
        rospy.init_node("block_detector")

        self.rgb_topic = rospy.get_param("~rgb_topic", "/camera/camera/color/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/camera/depth/image_raw")
        self.detection_topic = rospy.get_param("~detection_topic", "/coverage_block_search/detection")

        self.image_width = int(rospy.get_param("~image_width", 320))
        self.image_height = int(rospy.get_param("~image_height", 240))
        self.min_block_area = float(rospy.get_param("~min_block_area", 450.0))
        self.max_square_ratio = float(rospy.get_param("~max_square_ratio", 2.5))
        self.show_debug_window = rospy.get_param("~show_debug_window", False)
        self.target_colors = read_color_list(rospy, "~target_colors", ["red", "blue", "green"])

        self.required_frames = int(rospy.get_param("~required_frames", 1))

        self.enable_depth_filter = rospy.get_param("~enable_depth_filter", True)
        self.depth_min_m = float(rospy.get_param("~depth_min_m", 0.10))
        self.depth_max_m = float(rospy.get_param("~depth_max_m", 2.00))
        self.depth_target_roi_scale = float(rospy.get_param("~depth_target_roi_scale", 0.55))
        self.depth_background_ring_scale = float(rospy.get_param("~depth_background_ring_scale", 2.00))
        self.depth_min_contrast_m = float(rospy.get_param("~depth_min_contrast_m", 0.04))
        self.depth_max_target_std_m = float(rospy.get_param("~depth_max_target_std_m", 0.08))
        self.depth_min_valid_pixels = int(rospy.get_param("~depth_min_valid_pixels", 20))
        self.depth_min_background_pixels = int(rospy.get_param("~depth_min_background_pixels", 40))
        self.depth_require_contrast = rospy.get_param("~depth_require_contrast", True)

        self.color_ranges = {
            "red": [
                (np.array([0, 100, 70]), np.array([10, 255, 255])),
                (np.array([170, 100, 70]), np.array([180, 255, 255])),
            ],
            "green": [(np.array([35, 45, 45]), np.array([90, 255, 255]))],
            "blue": [(np.array([95, 60, 45]), np.array([135, 255, 255]))],
            "yellow": [(np.array([18, 80, 80]), np.array([35, 255, 255]))],
        }
        self.kernel = np.ones((5, 5), np.uint8)

        self.bridge = CvBridge()

        self.depth_lock = threading.RLock()
        self.latest_depth = None
        self.latest_depth_stamp = None

        self.detection_pub = rospy.Publisher(self.detection_topic, String, queue_size=10)
        rospy.Subscriber(
            self.rgb_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.Subscriber(
            self.depth_topic,
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "Block detector started: rgb_topic=%s depth_topic=%s detection_topic=%s depth_filter=%s",
            self.rgb_topic,
            self.depth_topic,
            self.detection_topic,
            self.enable_depth_filter,
        )

    def depth_callback(self, msg):
        depth = self.ros_depth_to_meters(msg)
        if depth is None:
            return
        with self.depth_lock:
            self.latest_depth = depth
            self.latest_depth_stamp = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()

    def image_callback(self, msg):
        frame = self.ros_image_to_bgr(msg)
        if frame is None:
            return

        detection = self.detect_colored_block(frame)

        if detection is not None:
            color = detection["color"]
            detection["stamp"] = msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec()
            detection["source"] = "hsv_depth_detector" if self.enable_depth_filter else "hsv_color_detector"
            self.detection_pub.publish(String(data=json.dumps(detection, sort_keys=True)))
            rospy.loginfo("DETECTED block: %s (area=%.0f)", color, detection["area"])

        if self.show_debug_window:
            self.show_debug(frame, detection)

    def ros_image_to_bgr(self, msg):
        try:
            encoding = msg.encoding.lower()

            if encoding in ("bgr8", "rgb8"):
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                if encoding == "rgb8":
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                return image.copy()

            if encoding in ("bgra8", "rgba8"):
                image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                if encoding == "rgba8":
                    return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
                return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

            # Fallback: use cv_bridge for any other encoding
            # (yuv422, bayer, 16-bit, etc. from Orbbec/other cameras)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            return cv_image.copy()
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Cannot convert image (encoding=%s): %s",
                                   msg.encoding, exc)
            return None

    def ros_depth_to_meters(self, msg):
        try:
            encoding = msg.encoding.lower()
            if encoding in ("16uc1", "mono16"):
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32)
                depth *= 0.001
            elif encoding == "32fc1":
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).astype(np.float32)
            else:
                rospy.logwarn_throttle(2.0, "Unsupported depth encoding: %s", msg.encoding)
                return None

            depth[depth <= 0.0] = np.nan
            depth[~np.isfinite(depth)] = np.nan
            return depth
        except ValueError as exc:
            rospy.logwarn_throttle(2.0, "Cannot convert depth image: %s", exc)
            return None

    def detect_colored_block(self, frame):
        resized = cv2.resize(frame, (self.image_width, self.image_height))
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        best = None
        for color in self.target_colors:
            if color not in self.color_ranges:
                continue

            mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)
            for lower, upper in self.color_ranges[color]:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < self.min_block_area:
                    continue

                rect = cv2.minAreaRect(contour)
                (cx, cy), (width, height), _ = rect
                short_side = min(width, height)
                long_side = max(width, height)
                if short_side <= 0:
                    continue

                square_ratio = long_side / short_side
                if square_ratio > self.max_square_ratio:
                    continue

                (_, _), radius = cv2.minEnclosingCircle(contour)
                candidate = {
                    "color": color,
                    "area": area,
                    "center_x": cx * frame.shape[1] / self.image_width,
                    "center_y": cy * frame.shape[0] / self.image_height,
                    "radius": radius * frame.shape[1] / self.image_width,
                    "square_ratio": square_ratio,
                    "score": area / max(1.0, square_ratio),
                }

                depth_ok, depth_info = self.validate_depth(candidate, frame.shape)
                if not depth_ok:
                    continue
                candidate.update(depth_info)

                if best is None or candidate["score"] > best["score"]:
                    best = candidate

        return best

    def validate_depth(self, candidate, frame_shape):
        if not self.enable_depth_filter:
            return True, {}

        with self.depth_lock:
            depth = None if self.latest_depth is None else self.latest_depth.copy()

        if depth is None:
            rospy.logwarn_throttle(2.0, "Waiting for depth image before accepting color detections")
            return False, {}

        frame_h, frame_w = frame_shape[:2]
        depth_h, depth_w = depth.shape[:2]
        cx = int(round(candidate["center_x"] * depth_w / frame_w))
        cy = int(round(candidate["center_y"] * depth_h / frame_h))
        radius = max(3, int(round(candidate["radius"] * depth_w / frame_w)))

        inner_r = max(3, int(round(radius * self.depth_target_roi_scale)))
        outer_r = max(inner_r + 3, int(round(radius * self.depth_background_ring_scale)))

        y_grid, x_grid = np.ogrid[:depth_h, :depth_w]
        dist2 = (x_grid - cx) ** 2 + (y_grid - cy) ** 2
        inner_mask = dist2 <= inner_r ** 2
        ring_mask = (dist2 > inner_r ** 2) & (dist2 <= outer_r ** 2)

        target_values = depth[inner_mask]
        target_values = target_values[np.isfinite(target_values)]
        target_values = target_values[(target_values >= self.depth_min_m) & (target_values <= self.depth_max_m)]
        if target_values.size < self.depth_min_valid_pixels:
            rospy.logdebug("Reject %s: too few valid target depth pixels", candidate["color"])
            return False, {}

        target_depth = float(np.median(target_values))
        target_std = float(np.std(target_values))
        if target_std > self.depth_max_target_std_m:
            rospy.logdebug("Reject %s: target depth std %.3fm too high", candidate["color"], target_std)
            return False, {}

        background_values = depth[ring_mask]
        background_values = background_values[np.isfinite(background_values)]
        background_values = background_values[(background_values >= self.depth_min_m) & (background_values <= self.depth_max_m)]

        background_depth = float("nan")
        contrast = float("nan")
        if background_values.size >= self.depth_min_background_pixels:
            background_depth = float(np.median(background_values))
            contrast = background_depth - target_depth
            if contrast < self.depth_min_contrast_m:
                rospy.logdebug(
                    "Reject %s: depth contrast %.3fm below %.3fm",
                    candidate["color"],
                    contrast,
                    self.depth_min_contrast_m,
                )
                return False, {}
        elif self.depth_require_contrast:
            rospy.logdebug("Reject %s: too few background depth pixels", candidate["color"])
            return False, {}

        return True, {
            "depth_m": target_depth,
            "depth_std_m": target_std,
            "background_depth_m": background_depth,
            "depth_contrast_m": contrast,
        }

    def show_debug(self, frame, detection):
        debug_frame = frame.copy()
        if detection is not None:
            color = detection["color"]
            center = (int(detection["center_x"]), int(detection["center_y"]))
            cv2.circle(debug_frame, center, int(detection["radius"]), (0, 255, 255), 2)
            label = color
            if "depth_m" in detection:
                label = "%s %.2fm" % (color, detection["depth_m"])
            cv2.putText(debug_frame, label, center, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("block_detector", debug_frame)
        cv2.waitKey(1)

    def on_shutdown(self):
        if self.show_debug_window:
            cv2.destroyAllWindows()


def main():
    BlockDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
