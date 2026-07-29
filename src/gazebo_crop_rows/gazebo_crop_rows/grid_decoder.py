#!/usr/bin/env python3
"""
grid_decoder.py — decode robot map position from downward camera image.

Grid: 21 cols x 22 rows, cell=1.0m, origin=(-2.0,-20.0).
Hue (0-300°) -> col (x), brightness (0.47-1.0) -> row (y).

Usage:
    from grid_decoder import decode_position
    map_x, map_y, confidence = decode_position(bgr_image)
"""

import cv2, numpy as np, math

GRID_X0   = -2.0
GRID_Y0   = -20.0
CELL      = 1.0
COLS      = 21
ROWS      = 22
HUE_MAX   = 300.0          # degrees (OpenCV hue = deg/2)
VAL_MIN   = 0.47
VAL_MAX   = 1.00

def decode_position(bgr):
    """
    Returns (map_x, map_y, confidence) where confidence in [0,1].
    map_x/map_y are cell-centre coordinates in map frame.
    Returns (None, None, 0) if no grid tile detected.
    """
    h, w = bgr.shape[:2]
    # Sample the ground zone: below toolbar, above rear wheels
    # Toolbar ~rows 45-80, wheels ~rows 200-240 in 320x240 image
    roi = bgr[80:200, 40:280]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV).astype(np.float32)

    # Mask: exclude near-white (sky), near-black, and low-saturation (ground brown)
    # Grid tiles: sat > 100, val > 60
    sat = hsv[:,:,1]
    val = hsv[:,:,2]
    mask = (sat > 100) & (val > 60)

    if mask.sum() < 50:
        return None, None, 0.0

    hues = hsv[:,:,0][mask] * 2.0        # OpenCV hue is deg/2 -> back to degrees
    vals = hsv[:,:,2][mask] / 255.0      # normalize to [0,1]

    # Median is robust to edge pixels from adjacent cells
    med_hue = float(np.median(hues))
    med_val = float(np.median(vals))

    # Decode col and row
    col = int(round(med_hue / (HUE_MAX / (COLS - 1))))
    col = max(0, min(COLS - 1, col))

    row = int(round((med_val - VAL_MIN) / ((VAL_MAX - VAL_MIN) / (ROWS - 1))))
    row = max(0, min(ROWS - 1, row))

    # Cell centre in map frame
    map_x = GRID_X0 + col * CELL + CELL / 2.0
    map_y = GRID_Y0 + row * CELL + CELL / 2.0

    # Confidence: fraction of masked pixels that agree with decoded cell color
    exp_hue = col * (HUE_MAX / (COLS - 1))
    exp_val = VAL_MIN + row * ((VAL_MAX - VAL_MIN) / (ROWS - 1))
    hue_err = np.abs(hues - exp_hue)
    val_err = np.abs(vals - exp_val)
    agreeing = ((hue_err < 15) & (val_err < 0.06)).sum()
    confidence = float(agreeing) / max(1, mask.sum())

    return map_x, map_y, confidence


if __name__ == '__main__':
    import sys
    img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else None
    if img is None:
        print("Usage: grid_decoder.py <image.jpg>")
        raise SystemExit(1)
    x, y, conf = decode_position(img)
    print(f"map_x={x:.2f}  map_y={y:.2f}  confidence={conf:.2f}")
