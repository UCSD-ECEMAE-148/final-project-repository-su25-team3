#!/usr/bin/env python3
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32

from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# Best-effort QoS for camera images
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

class SafetyStopYOLO(Node):
    def __init__(self):
        super().__init__('safety_stop_yolo')

        # ---- ROS parameters ----
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('obstruction_topic', '/obstruction_active')
        self.declare_parameter('distance_topic', '/obstacle_distance')
        self.declare_parameter('min_trigger_interval_s', 0.5)

        self.image_topic = self.get_parameter('image_topic').value
        self.obstruction_topic = self.get_parameter('obstruction_topic').value
        self.distance_topic = self.get_parameter('distance_topic').value
        self.min_trigger_interval_s = float(self.get_parameter('min_trigger_interval_s').value)

        # ---- state ----
        self.bridge = CvBridge()
        self._armed_for_one_shot = False
        self._last_trigger_time = 0.0
        self._prev_obstructed = False
        self._last_distance = None  # Float32 value

        # ---- Load YOLO OpenVINO model ----
        self.model_path = "/home/projects/ros2_ws/src/safety_stop/safety_stop/weights/yolov8s_openvino_model"
        self.get_logger().info(f"Loading YOLOv8 OpenVINO model from {self.model_path}...")
        self.model = YOLO(self.model_path)
        self.get_logger().info("YOLOv8 OpenVINO model loaded ✅")

        # ---- Subscribers ----
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile=sensor_qos)
        self.create_subscription(Bool, self.obstruction_topic, self.on_obstruction, qos_profile=10)
        self.create_subscription(Float32, self.distance_topic, self.on_distance, qos_profile=10)

    # Keep track of last distance
    def on_distance(self, msg: Float32):
        self._last_distance = msg.data

    # Rising-edge obstruction trigger
    def on_obstruction(self, msg: Bool):
        now = time.time()
        obstructed = bool(msg.data)

        if (not self._prev_obstructed) and obstructed:
            if now - self._last_trigger_time >= self.min_trigger_interval_s:
                self._armed_for_one_shot = True
                self._last_trigger_time = now
                d_str = f"{self._last_distance:.2f} m" if self._last_distance is not None else "unknown"
                self.get_logger().info(f"Obstruction ACTIVE → will classify next frame (last distance: {d_str})")
        self._prev_obstructed = obstructed

    # Run YOLO inference on the triggered frame
    def on_image(self, msg: Image):
        if not self._armed_for_one_shot:
            return
        self._armed_for_one_shot = False  # consume trigger

        # Convert ROS Image -> BGR OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        try:
            results = self.model(frame,imgsz=960,conf=0.4)  # OpenVINO inference
            summary = self._format_results(results)
            d_str = f"{self._last_distance:.2f} m" if self._last_distance else "unknown"
            self.get_logger().info(f"{summary} | last distance: {d_str}")
        except Exception as e:
            self.get_logger().warn(f"YOLO OpenVINO inference failed: {e}")

    # Pretty-print results
    def _format_results(self, results) -> str:
        if not results or len(results) == 0:
            return "YOLO: no predictions"
        try:
            r = results[0]
            if hasattr(r, 'boxes') and r.boxes is not None and len(r.boxes) > 0:
                top_pred = r.boxes.cls[0].item() if hasattr(r.boxes, 'cls') else None
                conf = r.boxes.conf[0].item() if hasattr(r.boxes, 'conf') else None
                return f"YOLO: class {top_pred} ({conf:.2f})"
            return "YOLO: no detected boxes"
        except Exception:
            return "YOLO: result parsing failed"

def main():
    rclpy.init()
    node = SafetyStopYOLO()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
