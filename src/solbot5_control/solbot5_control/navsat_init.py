#!/usr/bin/env python3
"""
navsat_init — wait for valid GPS fix, then set navsat_transform datum.

navsat_transform is started with wait_for_datum=true. This node waits
for a valid GPS fix, then calls /datum with the current position and
identity orientation (heading=0). The EKF handles heading via GPS
velocity fusion (imu1/gps_heading).

Subscribes:
    /gps/fix — sensor_msgs/NavSatFix (waits for valid fix)

Calls:
    /datum — robot_localization/srv/SetDatum
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix


class NavsatInit(Node):

    def __init__(self):
        super().__init__('navsat_init')

        self._datum_set = False

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(
            NavSatFix, '/gps/fix', self._cb_fix, best_effort)

        self.get_logger().info('navsat_init: waiting for valid GPS fix to set datum...')

    def _cb_fix(self, msg):
        if self._datum_set:
            return
        if msg.status.status < 0:  # no fix
            return
        self._set_datum(msg)

    def _set_datum(self, fix):
        from robot_localization.srv import SetDatum
        from geographic_msgs.msg import GeoPose, GeoPoint
        from geometry_msgs.msg import Quaternion

        client = self.create_client(SetDatum, '/datum')
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn(
                'navsat_init: /datum service not yet available — will retry on next fix')
            self._datum_set = False
            return

        # Zero heading — EKF handles orientation via GPS velocity heading (imu1)
        req = SetDatum.Request()
        req.geo_pose = GeoPose()
        req.geo_pose.position = GeoPoint(
            latitude=fix.latitude,
            longitude=fix.longitude,
            altitude=fix.altitude)
        req.geo_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        future = client.call_async(req)
        future.add_done_callback(self._datum_done)
        self._datum_set = True

        self.get_logger().info(
            f'navsat_init: setting datum at '
            f'({fix.latitude:.8f}, {fix.longitude:.8f})')

    def _datum_done(self, future):
        try:
            future.result()
            self.get_logger().info('navsat_init: datum set — navsat_transform active')
        except Exception as e:
            self.get_logger().error(f'navsat_init: datum call failed: {e}')
            self._datum_set = False


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavsatInit()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
