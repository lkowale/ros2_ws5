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
    """Set navsat_transform's /datum from the first valid GPS fix.

    Persistently retries via a timer until the SetDatum call succeeds, so a
    standalone localization restart (navsat_transform comes up datum-less) is
    reliably re-initialized even if /datum is not yet available when the first
    fix arrives.
    """

    def __init__(self):
        super().__init__('navsat_init')

        from robot_localization.srv import SetDatum
        self._SetDatum = SetDatum

        self._datum_confirmed = False   # SetDatum call returned successfully
        self._call_pending = False      # a SetDatum future is in flight
        self._last_fix = None           # latest valid fix to use as datum

        best_effort = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            NavSatFix, '/gps/fix', self._cb_fix, best_effort)

        self._client = self.create_client(SetDatum, '/datum')

        # Drive the datum-setting from a timer (non-blocking, retries until done).
        self._timer = self.create_timer(1.0, self._try_set_datum)

        self.get_logger().info('navsat_init: waiting for valid GPS fix to set datum...')

    def _cb_fix(self, msg):
        if msg.status.status < 0:  # no fix
            return
        self._last_fix = msg

    def _try_set_datum(self):
        if self._datum_confirmed or self._call_pending:
            return
        if self._last_fix is None:
            return
        if not self._client.service_is_ready():
            # navsat_transform not up yet — keep waiting, timer will retry.
            return

        from geographic_msgs.msg import GeoPose, GeoPoint
        from geometry_msgs.msg import Quaternion

        fix = self._last_fix
        # Zero heading — orientation is supplied to the EKF separately
        # (dual-antenna heading on /imu/gps_heading).
        req = self._SetDatum.Request()
        req.geo_pose = GeoPose()
        req.geo_pose.position = GeoPoint(
            latitude=fix.latitude,
            longitude=fix.longitude,
            altitude=fix.altitude)
        req.geo_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self._call_pending = True
        self.get_logger().info(
            f'navsat_init: setting datum at '
            f'({fix.latitude:.8f}, {fix.longitude:.8f})')
        self._client.call_async(req).add_done_callback(self._datum_done)

    def _datum_done(self, future):
        self._call_pending = False
        try:
            future.result()
            self._datum_confirmed = True
            self.get_logger().info('navsat_init: datum set — navsat_transform active')
            self._timer.cancel()
        except Exception as e:
            self.get_logger().warn(f'navsat_init: datum call failed ({e}) — retrying')


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
