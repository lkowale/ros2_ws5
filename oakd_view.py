#!/usr/bin/env python3
"""Laptop viewer for the solbot4 OAK-D stream: subscribes to a sensor_msgs/CompressedImage
topic, decodes the JPEG and shows it with OpenCV (q / Esc closes). Received rate in the corner.

Started by `bash pd600.sh view` (domain 1 over Tailscale). Usage:
    python3 oakd_view.py [topic]      default /oakd/rgb/image_raw/compressed
"""

import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class OakdView(Node):
    def __init__(self, topic):
        super().__init__('oakd_view')
        self.frame = None
        self.stamps = []
        # best effort matches reliable and best-effort publishers
        self.create_subscription(CompressedImage, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f'waiting for {topic}')

    def on_image(self, msg):
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        now = time.monotonic()
        self.stamps = [t for t in self.stamps if now - t < 2.0] + [now]
        hz = (len(self.stamps) - 1) / (self.stamps[-1] - self.stamps[0]) if len(self.stamps) > 1 else 0.0
        cv2.putText(img, f'{hz:4.1f} Hz  {len(msg.data) // 1024} kB', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        if self.frame is None:
            self.get_logger().info(f'first image {img.shape[1]}x{img.shape[0]}, {len(msg.data) // 1024} kB')
        self.frame = img


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/oakd/rgb/image_raw/compressed'
    rclpy.init()
    node = OakdView(topic)
    win = 'oakd ' + topic
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    placeholder = np.zeros((360, 640, 3), np.uint8)
    cv2.putText(placeholder, 'waiting for images...', (150, 180),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 200, 200), 2)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            cv2.imshow(win, node.frame if node.frame is not None else placeholder)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
