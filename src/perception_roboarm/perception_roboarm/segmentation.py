#!/usr/bin/env python3
"""Detect red, pink and green regions and display their 2D pixel coordinates.

Run with ROS 2 Jazzy sourced and Gazebo plus the camera bridge running:
    python3 display_node_2d.py

Optional topic override:
    python3 display_node_2d.py --ros-args -p image_topic:=/your/image/topic

Outputs: three OpenCV windows, a terminal summary once per second, and
self.detections containing each accepted region's mask, centroid and pixels.
All coordinates refer to the ORIGINAL image, even if a window is resized.
This node does not compute depth, 3D coordinates, or robot motion.
"""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('min_area', 500.0)
        self.image_topic = self.get_parameter('image_topic').value
        self.min_area = float(self.get_parameter('min_area').value)

        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), dtype=np.uint8)
        self.detections = []
        self.image_header = None
        self.stop_requested = False
        self.windows_created = False
        self.last_log_time = None

        self.display_subscriber = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Waiting for images on {self.image_topic}. '
            'Windows appear when the first frame arrives. '
            'Press q in an image window, or Ctrl+C in this terminal, to stop.'
        )
    @staticmethod
    def create_color_masks(hsv_image):
        """Use the same HSV thresholds as your current code."""
        pink_mask = cv2.inRange(
            hsv_image, np.array([154, 165, 95]), np.array([164, 178, 219])
        )
        red_low = cv2.inRange(
            hsv_image, np.array([0, 100, 80]), np.array([10, 255, 255])
        )
        red_high = cv2.inRange(
            hsv_image, np.array([170, 100, 80]), np.array([179, 255, 255])
        )
        green_mask = cv2.inRange(
            hsv_image, np.array([64, 157, 92]), np.array([74, 176, 215])
        )
        return {
            'red': cv2.bitwise_or(red_low, red_high),
            'pink': pink_mask,
            'green': green_mask,
        }

    def extract_detections(self, color_masks):
        """Extract each separate region, its pixel list and its centroid."""
        detections = []

        for color_name, color_mask in color_masks.items():
            clean_mask = cv2.morphologyEx(
                color_mask, cv2.MORPH_OPEN, self.kernel
            )
            contours, _ = cv2.findContours(
                clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # Left-to-right ordering makes the display easier to inspect.
            # These numbers are per-frame labels, not persistent tracking IDs.
            contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])
            region_number = 0

            for contour in contours:
                if cv2.contourArea(contour) <= self.min_area:
                    continue

                object_mask = np.zeros_like(clean_mask)
                cv2.drawContours(
                    object_mask, [contour], -1, 255, cv2.FILLED
                )
                # Preserve holes: include only the originally detected pixels.
                object_mask = cv2.bitwise_and(object_mask, clean_mask)

                # NumPy returns rows first (v), then columns (u).
                v, u = np.nonzero(object_mask)
                if u.size == 0:
                    continue

                region_number += 1
                detections.append({
                    'name': f'{color_name}#{region_number}',
                    'color': color_name,
                    'centroid_px': (float(u.mean()), float(v.mean())),
                    'pixels_uv': np.column_stack((u, v)),
                    'pixel_count': int(u.size),
                    'bbox_xywh': cv2.boundingRect(contour),
                    'mask': object_mask,
                })

        return detections

    @staticmethod
    def create_output_images(cv_image, detections):
        """Build a binary mask and an annotated color image on black."""
        accepted_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
        for detection in detections:
            accepted_mask = cv2.bitwise_or(
                accepted_mask, detection['mask']
            )

        # Object colors are preserved. The background is black.
        result = cv2.bitwise_and(cv_image, cv_image, mask=accepted_mask)

        for detection in detections:
            u_center, v_center = detection['centroid_px']
            x, y, w, h = detection['bbox_xywh']
            center = (int(round(u_center)), int(round(v_center)))

            cv2.rectangle(result, (x, y), (x + w - 1, y + h - 1),
                          (255, 255, 255), 1)
            cv2.circle(result, center, 6, (0, 0, 0), -1)
            cv2.circle(result, center, 4, (255, 255, 255), -1)

            label = (f"{detection['name']} "
                     f"u={u_center:.1f}, v={v_center:.1f}")
            font = cv2.FONT_HERSHEY_SIMPLEX
            scale = 0.45
            text_width = cv2.getTextSize(label, font, scale, 1)[0][0]
            label_x = max(0, min(x, cv_image.shape[1] - text_width - 2))
            label_y = max(18, y - 8)
            # Dark outline keeps the white label legible over objects.
            cv2.putText(result, label, (label_x, label_y), font, scale,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(result, label, (label_x, label_y), font, scale,
                        (255, 255, 255), 1, cv2.LINE_AA)

        return accepted_mask, result

    def log_detections(self, cv_image):
        """Print readable numerical output without flooding the terminal."""
        now = time.monotonic()
        if self.last_log_time is not None and now - self.last_log_time < 1.0:
            return
        self.last_log_time = now

        height, width = cv_image.shape[:2]
        lines = [f'Image: {width} x {height}; detections: {len(self.detections)}']
        for detection in self.detections:
            u, v = detection['centroid_px']
            lines.append(
                f"  {detection['name']}: centroid(u,v)=({u:.1f}, {v:.1f}) px; "
                f"pixels={detection['pixel_count']}; "
                f"bbox(x,y,w,h)={detection['bbox_xywh']}"
            )
        if not self.detections:
            lines.append('  No region passed the HSV, morphology and area filters.')
        self.get_logger().info('\n'.join(lines))

    def image_callback(self, msg):
        """Run once for each received ROS image, not on a timer."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'cv_bridge error: {exc}')
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        color_masks = self.create_color_masks(hsv_image)
        self.detections = self.extract_detections(color_masks)
        self.image_header = msg.header
        accepted_mask, result = self.create_output_images(
            cv_image, self.detections
        )

        if not self.windows_created:
            for name in ['1 - Original image', '2 - Accepted binary mask',
                         '3 - Objects and pixel coordinates']:
                cv2.namedWindow(name, cv2.WINDOW_NORMAL)
                height, width = cv_image.shape[:2]
                preview_width = min(width, 800)
                preview_height = max(1, round(height * preview_width / width))
                cv2.resizeWindow(name, preview_width, preview_height)
            self.windows_created = True

        cv2.imshow('1 - Original image', cv_image)
        cv2.imshow('2 - Accepted binary mask', accepted_mask)
        cv2.imshow('3 - Objects and pixel coordinates', result)
        self.log_detections(cv_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_requested = True

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SegmentationNode()
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
