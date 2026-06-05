#!/usr/bin/env python3
"""
polygon_recorder — Record field boundary polygon points via MQTT.

Workflow:
  1. Drive robot antenna to a corner point.
  2. Detach antenna, place on ground, press 'Record Point' on Android MQTT client.
     → Publishes to MQTT: outer/record_point  {"data": 1}
  3. Robot publishes confirmation: robot/point_recorded {"index": N, "lat": ..., "lon": ...}
  4. Re-attach antenna, drive to next corner.
  5. After all corners: send outer/save_polygon {"data": "my_field_name"}
     → Writes GeoJSON polygon to ~/ros2_ws4/src/fields/<name>/<name>_polygon.geojson
     → Confirms on robot/polygon_saved

MQTT commands (subscribe):
  outer/record_point   {"data": 1}              — capture current GPS position
  outer/clear_polygon  {"data": 1}              — discard all recorded points
  outer/save_polygon   {"data": "field_name"}   — write GeoJSON file and confirm

MQTT status (publish):
  robot/point_recorded   — confirmation per point
  robot/polygon_status   — periodic status (point count, last point)
  robot/polygon_saved    — confirmation when file written (or error)
"""

import os
import json
import math
import time
import uuid

import rclpy
import rclpy.executors
import paho.mqtt.client as mqtt

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


FIELDS_DIR = os.path.expanduser('~/ros2_ws4/src/fields')


class PolygonRecorder(Node):
    def __init__(self):
        super().__init__('polygon_recorder')

        # --- GPS state ---
        self.latest_fix: NavSatFix | None = None
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(NavSatFix, '/gps/fix', self._on_gps, qos)

        # --- Recorded points (lon, lat) — GeoJSON order ---
        self.points: list[tuple[float, float]] = []

        # --- Field name (set via outer/field_name, used by outer/save_polygon) ---
        self.field_name: str = ''

        # --- MQTT ---
        unique_id = str(uuid.uuid4())[:8]
        self.mqttclient = mqtt.Client(
            client_id=f'polygon_recorder_{unique_id}',
            userdata=None,
            protocol=mqtt.MQTTv5,
        )
        self.mqttclient.on_connect = self._on_mqtt_connect
        self.mqttclient.on_disconnect = self._on_mqtt_disconnect
        self.mqttclient.on_message = self._on_mqtt_message
        self.mqttclient.tls_set(tls_version=mqtt.ssl.PROTOCOL_TLS)
        self.mqttclient.username_pw_set(username='aargideon', password='para!234')
        self.mqttclient.connect('ae4cb1b10ad84e53af8887dd32476b04.s2.eu.hivemq.cloud', 8883)
        self.mqttclient.subscribe('outer/#')
        self.mqttclient.loop_start()

        # Periodic status at 0.2 Hz
        self.create_timer(5.0, self._publish_status)

        self._shutdown_requested = False
        self.get_logger().info('polygon_recorder started. Waiting for GPS and MQTT commands.')

    # ------------------------------------------------------------------ #
    #  GPS                                                                 #
    # ------------------------------------------------------------------ #

    def _on_gps(self, msg: NavSatFix):
        self.latest_fix = msg

    def _gps_ok(self) -> tuple[bool, str]:
        """Return (ok, reason). ok=True when GPS is usable for recording."""
        if self.latest_fix is None:
            return False, 'No GPS fix received yet'
        if self.latest_fix.status.status < 0:
            return False, f'GPS status={self.latest_fix.status.status} (no fix)'
        return True, ''

    def _gps_quality_note(self) -> str:
        """Human-readable accuracy note; empty string = no warning."""
        fix = self.latest_fix
        if fix is None:
            return ''
        cov = fix.position_covariance
        if fix.position_covariance_type > 0 and cov[0] > 0:
            h_sigma = math.sqrt(cov[0])
            if h_sigma > 0.015:
                return f'low accuracy ({h_sigma * 1000:.1f} mm 1-sigma, need <15 mm)'
        return ''

    # ------------------------------------------------------------------ #
    #  MQTT callbacks                                                      #
    # ------------------------------------------------------------------ #

    def _on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            self.get_logger().info('polygon_recorder: connected to MQTT broker')
        else:
            self.get_logger().error(f'polygon_recorder: MQTT connect failed rc={rc}')

    def _on_mqtt_disconnect(self, client, userdata, flags, reason_code, properties=None):
        if self._shutdown_requested:
            return
        self.get_logger().warn(f'polygon_recorder: MQTT disconnected ({reason_code}), reconnecting…')
        delay = 1
        for _ in range(12):
            if self._shutdown_requested:
                return
            time.sleep(delay)
            try:
                client.reconnect()
                self.get_logger().info('polygon_recorder: MQTT reconnected')
                return
            except Exception as e:
                self.get_logger().warn(f'polygon_recorder: reconnect failed: {e}')
            delay = min(delay * 2, 60)

    def _on_mqtt_message(self, client, userdata, msg):
        topic = msg.topic
        try:
            payload = json.loads(msg.payload.decode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'polygon_recorder: bad JSON on {topic}: {e}')
            return

        if topic == 'outer/record_point':
            self._cmd_record_point(payload)
        elif topic == 'outer/field_name':
            self._cmd_set_field_name(payload)
        elif topic == 'outer/clear_polygon':
            self._cmd_clear(payload)
        elif topic == 'outer/save_polygon':
            self._cmd_save(payload)

    # ------------------------------------------------------------------ #
    #  Commands                                                            #
    # ------------------------------------------------------------------ #

    def _cmd_record_point(self, payload):
        """Capture current GPS position as next polygon vertex."""
        ok, reason = self._gps_ok()
        if not ok:
            self._publish('robot/point_recorded', {
                'success': False,
                'error': reason,
                'point_count': len(self.points),
            })
            self.get_logger().warn(f'Record point rejected: {reason}')
            return

        fix = self.latest_fix
        lat = fix.latitude
        lon = fix.longitude
        self.points.append((lon, lat))   # GeoJSON: [longitude, latitude]
        index = len(self.points)

        note = self._gps_quality_note()
        response: dict = {
            'success': True,
            'index': index,
            'lat': round(lat, 9),
            'lon': round(lon, 9),
            'point_count': index,
        }
        if note:
            response['warning'] = note

        self._publish('robot/point_recorded', response)
        self.get_logger().info(
            f'Point {index} recorded: lat={lat:.8f} lon={lon:.8f}'
            + (f'  [{note}]' if note else '')
        )

    def _cmd_set_field_name(self, payload):
        """Store the field name typed by the user."""
        name = str(payload.get('data', '')).strip()
        self.field_name = name
        self._publish('robot/polygon_status', {
            'action': 'field_name_set',
            'field_name': self.field_name,
            'point_count': len(self.points),
        })
        self.get_logger().info(f'Field name set to: "{self.field_name}"')

    def _cmd_clear(self, payload):
        """Discard all recorded points."""
        count = len(self.points)
        self.points.clear()
        self._publish('robot/polygon_status', {
            'action': 'cleared',
            'discarded_points': count,
            'point_count': 0,
        })
        self.get_logger().info(f'Polygon cleared ({count} points discarded)')

    def _cmd_save(self, payload):
        """Write GeoJSON polygon file for the named field."""
        # Accept field name from payload, fall back to stored name
        field_name = str(payload.get('data', '')).strip() or self.field_name
        if not field_name:
            self._publish('robot/polygon_saved', {'success': False, 'error': 'field name not set — edit the Save tile payloadOn'})
            return

        n = len(self.points)
        if n < 3:
            self._publish('robot/polygon_saved', {
                'success': False,
                'error': f'Need at least 3 points, have {n}',
            })
            self.get_logger().warn(f'Save rejected: only {n} point(s) recorded')
            return

        # Close the ring: GeoJSON polygon requires first == last coordinate
        ring = list(self.points) + [self.points[0]]

        geojson = {
            'type': 'FeatureCollection',
            'features': [
                {
                    'type': 'Feature',
                    'properties': {'name': field_name},
                    'geometry': {
                        'type': 'Polygon',
                        'coordinates': [ring],
                    },
                }
            ],
        }

        field_dir = os.path.join(FIELDS_DIR, field_name)
        os.makedirs(field_dir, exist_ok=True)
        out_path = os.path.join(field_dir, f'{field_name}_polygon.geojson')

        try:
            with open(out_path, 'w') as f:
                json.dump(geojson, f, indent=2)
        except Exception as e:
            self._publish('robot/polygon_saved', {'success': False, 'error': str(e)})
            self.get_logger().error(f'Failed to write {out_path}: {e}')
            return

        self._publish('robot/polygon_saved', {
            'success': True,
            'field_name': field_name,
            'point_count': n,
            'path': out_path,
        })
        self.get_logger().info(f'Polygon saved: {out_path}  ({n} points)')

    # ------------------------------------------------------------------ #
    #  Periodic status                                                     #
    # ------------------------------------------------------------------ #

    def _publish_status(self):
        status: dict = {'point_count': len(self.points), 'field_name': self.field_name}
        ok, reason = self._gps_ok()
        status['gps_ok'] = ok
        if not ok:
            status['gps_error'] = reason
        if self.points:
            last_lon, last_lat = self.points[-1]
            status['last_lat'] = round(last_lat, 9)
            status['last_lon'] = round(last_lon, 9)
        self._publish('robot/polygon_status', status)

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def _publish(self, topic: str, payload: dict):
        try:
            self.mqttclient.publish(topic, json.dumps(payload), qos=1)
        except Exception as e:
            self.get_logger().error(f'MQTT publish failed on {topic}: {e}')

    def destroy_node(self):
        self._shutdown_requested = True
        self.mqttclient.loop_stop()
        self.mqttclient.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PolygonRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
