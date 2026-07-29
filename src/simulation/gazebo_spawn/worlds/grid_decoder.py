#!/usr/bin/env python3
"""
grid_decoder.py — decode robot gz_world position from downward camera image via QR code.

Grid: 68 cols x 48 rows, cell=0.5m, origin=(-4.0,-18.0).
Each floor cell contains a QR code encoding "col,row".

Ogre2 UV mapping for a box top face swaps the texture axes relative to world XY:
  QR col (texture U) maps to world Y; QR row (texture V) maps to world X.
  The U axis is also inverted: col=0 is world Y_max, col=COLS-1 is world Y_min.

Usage:
    from grid_decoder import decode_position
    gz_world_x, gz_world_y, confidence = decode_position(bgr_image)
"""

import cv2, numpy as np

GRID_X0 = -4.0
GRID_Y0 = -18.0
CELL    = 0.5
COLS    = 68
ROWS    = 48

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


def decode_position(bgr):
    """
    Returns (gz_world_x, gz_world_y, confidence) in Gazebo world frame.
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
                gz_world_x = GRID_X0 + row * CELL + CELL / 2.0
                gz_world_y = GRID_Y0 + (ROWS - 1 - col) * CELL + CELL / 2.0
                return gz_world_x, gz_world_y, 1.0
    return None, None, 0.0


if __name__ == '__main__':
    import sys
    img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else None
    if img is None:
        print("Usage: grid_decoder.py <image.jpg>")
        raise SystemExit(1)
    x, y, conf = decode_position(img)
    print(f"gz_world_x={x}  gz_world_y={y}  confidence={conf}")
