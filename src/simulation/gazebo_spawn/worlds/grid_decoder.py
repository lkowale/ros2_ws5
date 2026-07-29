#!/usr/bin/env python3
"""
grid_decoder.py — decode robot gz_world position from downward camera image via QR code.

Grid: 52 cols x 48 rows, cell=0.5m, origin=(-4.0,-18.0).
Each floor cell contains a QR code encoding "col,row".

Ogre2 UV convention for box top face: texture U axis maps to world Y (inverted),
texture V axis maps to world X. So decoded QR row→world_x, (ROWS-1-col)→world_y.

Usage:
    from grid_decoder import decode_position
    gz_world_x, gz_world_y, confidence = decode_position(bgr_image)
"""

import cv2, numpy as np

GRID_X0 = -4.0
GRID_Y0 = -18.0
CELL    = 0.5
COLS    = 52
ROWS    = 48

try:
    _wechat = cv2.wechat_qrcode_WeChatQRCode()
except Exception:
    _wechat = None
_detector = cv2.QRCodeDetector()

def _try_decode(img):
    """Return decoded QR string or '' on failure. Tries WeChatQR (4x) then standard."""
    # WeChatQR on 4x upscale is the most reliable at small QR sizes
    if _wechat is not None:
        big = cv2.resize(img, (img.shape[1]*4, img.shape[0]*4), interpolation=cv2.INTER_LINEAR)
        try:
            results, _ = _wechat.detectAndDecode(big)
            if results:
                return results[0]
        except Exception:
            pass
    # Fallback: standard detector
    try:
        val, pts, _ = _detector.detectAndDecode(img)
        if val:
            return val
    except cv2.error:
        pass
    return ''


def decode_position(bgr):
    """
    Returns (gz_world_x, gz_world_y, confidence) in Gazebo world frame.
    confidence=1.0 on success, 0.0 on failure.

    Ogre2 box UV mapping swaps X/Y relative to the world frame:
      - QR 'col,row': texture col (U axis) maps to world Y, row (V axis) maps to world X.
      - The V axis (row) is not flipped: row=0 → world_x=GRID_X0 (west edge).
      - The U axis (col) is flipped: col=0 → world_y=GRID_Y0+ROWS*CELL (north),
        col=COLS-1 → world_y=GRID_Y0 (south). So: world_y = GRID_Y0 + (ROWS-1-col)*CELL + CELL/2.
    """
    h, w = bgr.shape[:2]
    rois = [
        bgr,              # full image — best when a QR is centered
        bgr[:70, :],      # above toolbar
        bgr[105:, :],     # below toolbar (includes wheels, but QR may be there)
        bgr[105:195, :],  # below toolbar, above wheels
    ]
    for roi in rois:
        if roi.size == 0:
            continue
        val = _try_decode(roi)
        if val:
            try:
                col, row = map(int, val.split(','))
            except ValueError:
                continue
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
