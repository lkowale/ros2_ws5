#!/usr/bin/env python3
"""
grid_decoder.py — decode robot gz_world position from downward camera image via QR code.

Grid: 68 cols x 48 rows, cell=0.5m, origin=(-4.0,-18.0).
Each floor cell contains a QR code encoding "col,row" where col is the X index (0..COLS-1)
and row is the Y index (0..ROWS-1).

Ogre2 UV mapping on the box top face inverts the U axis:
  col=0 → world X_max (east), col=COLS-1 → world X_min (west)
  row=0 → world Y_max (north), row=ROWS-1 → world Y_min (south)

Formula (empirically verified):
  gz_world_x = GRID_X0 + (COLS-1-col)*CELL + CELL/2
  gz_world_y = GRID_Y0 + (ROWS-1-row)*CELL + CELL/2

Camera correction (oakd_link at +0.5m forward, 0.8m height, rpy="0 pi/2 0"):
  The camera looks backward. The centre of what it sees is ~1.21m behind and ~0.67m
  to the right of base_footprint (body frame). Pass yaw_deg to decode_position() for
  a heading-corrected robot position estimate; omit for a west-heading static fallback.

Usage:
    from grid_decoder import decode_position
    gz_world_x, gz_world_y, confidence = decode_position(bgr_image)
"""

import cv2, math, numpy as np

GRID_X0 = -4.0
GRID_Y0 = -18.0
CELL    = 0.5
COLS    = 68
ROWS    = 48

# Camera look-behind in body frame (empirical, record13 west swath n=11896).
# The camera (rpy="0 pi/2 0") looks backward. The decoded tile centre is ~1.2m
# behind base_footprint (+X is forward, so negative = behind) and ~0.67m to the right.
# Signed convention: positive X = forward, positive Y = left (ROS body frame).
# tile_world = robot_world + R(yaw) * [CAM_BODY_X, CAM_BODY_Y]
# → robot_world = tile_world − R(yaw) * [CAM_BODY_X, CAM_BODY_Y]
_CAM_BODY_X = -1.21   # metres behind base_footprint  (negative = backward)
_CAM_BODY_Y = +0.67   # metres to the right (negative Y in standard ROS = right)

try:
    import zxingcpp as _zxing
    _HAS_ZXING = True
except ImportError:
    _HAS_ZXING = False

try:
    _wechat = cv2.wechat_qrcode_WeChatQRCode()
except Exception:
    _wechat = None
_detector = cv2.QRCodeDetector()


def _try_decode(img):
    big = cv2.resize(img, (img.shape[1]*4, img.shape[0]*4), interpolation=cv2.INTER_LINEAR)
    if _HAS_ZXING:
        try:
            results = _zxing.read_barcodes(big)
            if results:
                return results[0].text
        except Exception:
            pass
    if _wechat is not None:
        try:
            results, _ = _wechat.detectAndDecode(big)
            if results:
                return results[0]
        except Exception:
            pass
    try:
        val, pts, _ = _detector.detectAndDecode(big)
        if val:
            return val
    except cv2.error:
        pass
    return ''


def _tile_to_world(col, row):
    """Return floor-tile centre in Gazebo world frame (no camera correction)."""
    gz_x = GRID_X0 + (COLS - 1 - col) * CELL + CELL / 2.0
    gz_y = GRID_Y0 + (ROWS - 1 - row) * CELL + CELL / 2.0
    return gz_x, gz_y


def decode_position(bgr, yaw_deg=None):
    """
    Returns (gz_world_x, gz_world_y, confidence) — estimated robot position.

    yaw_deg: robot heading in degrees (0=east, 90=north). When supplied the
             camera body-frame offset is rotated to world frame and subtracted.
             When None the west-heading world offset is used as a fallback.
    confidence=1.0 on success, 0.0 on failure.
    """
    val = _try_decode(bgr)
    if val:
        try:
            col, row = map(int, val.split(','))
        except ValueError:
            pass
        else:
            if 0 <= col < COLS and 0 <= row < ROWS:
                tile_x, tile_y = _tile_to_world(col, row)
                if yaw_deg is not None:
                    yr = math.radians(yaw_deg)
                    # The camera looks backward; tile is behind the robot.
                    # tile_world = robot_world − R(yaw) * cam_body
                    # → robot_world = tile_world + R(yaw) * cam_body
                    off_x = _CAM_BODY_X * math.cos(yr) - _CAM_BODY_Y * math.sin(yr)
                    off_y = _CAM_BODY_X * math.sin(yr) + _CAM_BODY_Y * math.cos(yr)
                    return tile_x + off_x, tile_y + off_y, 1.0
                else:
                    # Static fallback (assumes west heading, yaw≈180°)
                    return tile_x - _CAM_BODY_X, tile_y - _CAM_BODY_Y, 1.0
    return None, None, 0.0


if __name__ == '__main__':
    import sys
    img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else None
    if img is None:
        print("Usage: grid_decoder.py <image.jpg>")
        raise SystemExit(1)
    x, y, conf = decode_position(img)
    print(f"gz_world_x={x}  gz_world_y={y}  confidence={conf}")
