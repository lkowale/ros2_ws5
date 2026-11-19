#!/usr/bin/env python3
"""Rope-following precision logger: camera-measured knife midpoint (pink markers on the knives' inner tips)
vs the rope column from /rope_follower/status, ~5 Hz.

Publishes /rope_follower/knife_markers (JSON) and writes OUT/precision_<stamp>.csv.
    rope_precision.py OUT_DIR [jpg]      (jpg: also save an annotated frame every 1 s while the lift is down)
Error mm uses the outdoor ground scale ~2.8 px/mm at the tip rows (shop 2.5 px/mm).
Marker thresholds: rope_follower.knife_markers (H 150-180, S > 40, V > 200).
"""
import json
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

PX_MM = 2.8


from rope_follower import knife_markers   # same detector as the follower (installed next to this script)


def pink_tips(img):
    """((ul, vl), (ur, vr)) of the two knife markers, or None (rope_follower.knife_markers, rows 330..620)."""
    r = knife_markers(img, 330, 620)
    return None if r is None else (r[3], r[4])


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/rope_logs/markers')
    save_jpg = len(sys.argv) > 2 and sys.argv[2] == 'jpg'
    os.makedirs(out, exist_ok=True)
    rclpy.init()
    n = rclpy.create_node('rope_precision')
    pub = n.create_publisher(String, '/rope_follower/knife_markers', 10)
    S = {'st': None, 'last': 0.0, 'save': 0.0}
    f = open(os.path.join(out, f'precision_{time.strftime("%Y%m%d_%H%M%S")}.csv'), 'w')
    f.write('t,lift,lost,tps,target_tps,active,u_rope,u_pred,u_model,u_meas,err_meas_px,err_meas_mm,'
            'err_model_px,gap_px\n')

    def on_st(m):
        S['st'] = json.loads(m.data)

    def on_img(m):
        t = time.time()
        if t - S['last'] < 0.2 or S['st'] is None:
            return
        S['last'] = t
        d = S['st']
        if d.get('gated', False):
            return                                         # lift not down
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        tp = pink_tips(img)
        ur = None if d.get('lost') else d.get('u_rope')
        um = d.get('u_knife')
        meas = gap = em = None
        if tp:
            (ul, _), (uR, _) = tp
            meas, gap = (ul + uR) / 2, uR - ul
            if ur is not None:
                em = ur - meas
        emm = None if em is None else round(em / PX_MM, 2)
        f.write(f'{t:.2f},{d.get("lift")},{d.get("lost")},{d.get("tps")},{d.get("target_tps")},{d.get("active")},'
                f'{ur},{d.get("u_pred")},{um},{meas},{em},{emm},'
                f'{None if (ur is None or um is None) else round(ur - um, 1)},{gap}\n')
        f.flush()
        pub.publish(String(data=json.dumps({
            'stamp': m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, 'lift': d.get('lift'),
            'lost': d.get('lost'), 'tps': d.get('tps'), 'target_tps': d.get('target_tps'), 'u_rope': ur,
            'u_pred': d.get('u_pred'), 'u_knife_model': um, 'u_knife_meas': meas,
            'u_marker_l': tp[0][0] if tp else None, 'u_marker_r': tp[1][0] if tp else None, 'gap_px': gap,
            'err_meas_px': em, 'err_meas_mm': emm, 'err_model_mm': d.get('err_mm')})))
        if save_jpg and t - S['save'] >= 1.0:
            S['save'] = t
            if tp:
                for u, v in tp:
                    cv2.circle(img, (int(u), int(v)), 14, (255, 0, 255), 3)
                cv2.line(img, (int(meas), 380), (int(meas), 580), (255, 0, 255), 3)
            if ur is not None:
                cv2.line(img, (int(ur), 380), (int(ur), 580), (0, 255, 0), 3)
            if um is not None:
                cv2.line(img, (int(um), 430), (int(um), 530), (255, 255, 0), 2)
            cv2.putText(img, f'TPS {d.get("tps")} meas err {"-" if emm is None else f"{emm:+.1f}mm"}',
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imwrite(os.path.join(out, f'{time.strftime("%H%M%S")}.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 80])

    n.create_subscription(String, '/rope_follower/status', on_st, 10)
    n.create_subscription(CompressedImage, '/oakd/rgb/image_raw/compressed', on_img, qos_profile_sensor_data)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        f.close()


if __name__ == '__main__':
    main()
