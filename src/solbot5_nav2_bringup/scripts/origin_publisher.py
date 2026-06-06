#!/usr/bin/env python3
"""
Origin Publisher for Mapviz
Publishes /local_xy_origin topic for swri_transform_util/Mapviz compatibility.

By default, waits for the first GPS fix on /gps/fix and uses that as the origin.
Falls back to parameter values if use_gps_origin is False.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


class OriginPublisher(Node):
    def __init__(self):
        super().__init__('origin_publisher')

        # Declare parameters for origin coordinates
        self.declare_parameter('origin_latitude', 53.5204991)
        self.declare_parameter('origin_longitude', 17.8258532)
        self.declare_parameter('origin_altitude', 100.0)
        self.declare_parameter('use_gps_origin', True)
        self.declare_parameter('gps_topic', '/gps/fix')

        self.use_gps = self.get_parameter('use_gps_origin').value

        # QoS profile for local_xy_origin - TRANSIENT_LOCAL so late subscribers get the message
        origin_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.origin_pub = self.create_publisher(
            PoseStamped,
            '/local_xy_origin',
            origin_qos
        )

        if self.use_gps:
            gps_topic = self.get_parameter('gps_topic').value
            gps_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            self.gps_sub = self.create_subscription(
                NavSatFix, gps_topic, self._gps_callback, gps_qos)
            self.get_logger().info(
                f'Waiting for first GPS fix on {gps_topic}...')
        else:
            self.origin_lat = self.get_parameter('origin_latitude').value
            self.origin_lon = self.get_parameter('origin_longitude').value
            self.origin_alt = self.get_parameter('origin_altitude').value
            self.publish_origin()
            self.timer = self.create_timer(10.0, self.publish_origin)
            self.get_logger().info(
                f'Origin Publisher started with origin: '
                f'lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}, '
                f'alt={self.origin_alt:.2f}')

    def _gps_callback(self, msg):
        if msg.status.status < 0:
            return  # no fix yet
        self.origin_lat = msg.latitude
        self.origin_lon = msg.longitude
        self.origin_alt = msg.altitude
        self.get_logger().info(
            f'Origin set from GPS fix: lat={self.origin_lat:.8f}, '
            f'lon={self.origin_lon:.8f}, alt={self.origin_alt:.2f}')
        # Unsubscribe — only need the first fix
        self.destroy_subscription(self.gps_sub)
        self.publish_origin()
        self.timer = self.create_timer(10.0, self.publish_origin)

    def publish_origin(self):
        origin_msg = PoseStamped()
        origin_msg.header.stamp = self.get_clock().now().to_msg()
        origin_msg.header.frame_id = 'map'
        origin_msg.pose.position.x = self.origin_lon
        origin_msg.pose.position.y = self.origin_lat
        origin_msg.pose.position.z = self.origin_alt
        origin_msg.pose.orientation.w = 1.0
        self.origin_pub.publish(origin_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OriginPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
