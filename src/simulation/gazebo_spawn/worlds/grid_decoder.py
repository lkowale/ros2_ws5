#!/usr/bin/env python3
"""
grid_decoder.py — decode robot gz_world position from downward camera image via QR code.

Grid: 68 cols x 48 rows, cell=0.5m, origin=(-4.0,-18.0). Floor box 34m×24m.
Each floor cell contains a QR code encoding "col,row" where col is the texture-U index
(0..COLS-1) and row is the texture-V index (0..ROWS-1).

Ogre2 box top-face UV mapping (empirically verified):
  row  → world X  (row=0 west, row=47 east), scale = FLOOR_W/ROWS = 34/48 ≈ 0.708 m/row
  col  → world -Y (col=0 north, col=67 south), scale = FLOOR_H/COLS = 24/68 ≈ 0.353 m/col

Formula:
  gz_world_x = GRID_X0 + row  * (FLOOR_W/ROWS) + (FLOOR_W/ROWS)/2
  gz_world_y = GRID_Y_MAX - (col + 0.5) * (FLOOR_H/COLS)

where GRID_Y_MAX = GRID_Y0 + FLOOR_H = 6.0 m (north edge of floor).

Camera (oakd_link): 0.5m ahead of base_footprint, 0.8m height, rpy="0 pi/2 0".
Boresight = straight down; image top = forward (+X_body); image right = left (+Y_body).
Pass yaw_deg (EKF heading in degrees, 0=east, 90=north) to get heading-corrected position.

Usage:
    from grid_decoder import decode_position
    gz_world_x, gz_world_y, confidence = decode_position(bgr_image, yaw_deg=yaw)
"""

import cv2, math, numpy as np

GRID_X0    = -4.0
GRID_Y0    = -18.0
CELL       = 0.5
COLS       = 68
ROWS       = 48
FLOOR_W    = COLS * CELL        # 34.0 m  (east-west extent)
FLOOR_H    = ROWS * CELL        # 24.0 m  (north-south extent)
GRID_Y_MAX = GRID_Y0 + FLOOR_H  # +6.0 m  (north edge)
ROW_SCALE  = FLOOR_W / ROWS     # 34/48 ≈ 0.7083 m/row  (world X per row step)
COL_SCALE  = FLOOR_H / COLS     # 24/68 ≈ 0.3529 m/col  (world Y per col step)

# Camera geometry (URDF oakd_link): 0.5m ahead of base_footprint, 0.8m above footprint.
# pitch=pi/2 boresight = -Z_body (straight down).
# Image top = +X_body (forward), image right = +Y_body (left of robot).
_CAM_BODY_X = +0.5    # camera nadir is 0.5m ahead of base_footprint
_CAM_BODY_Y =  0.0
_CAM_HEIGHT = 0.80    # m above footprint
_CAM_HFOV   = 1.414   # rad  (81 deg OAK-D Lite RGB)
_IMG_W      = 320
_IMG_H      = 240

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


def _decode_all(img):
    """Return list of (text, centre_x_norm, centre_y_norm) for every QR in img.

    centre_x/y are normalised image coordinates [0,1] of the QR bounding box
    centre, so callers can pick the code closest to the image centre (optical axis).
    """
    ih, iw = img.shape[:2]
    found = []
    if _HAS_ZXING:
        # zxingcpp works best on the original-resolution image; upscaling degrades it.
        try:
            for r in _zxing.read_barcodes(img):
                if r.text:
                    pts = r.position
                    xs = [pts.top_left.x, pts.top_right.x, pts.bottom_right.x, pts.bottom_left.x]
                    ys = [pts.top_left.y, pts.top_right.y, pts.bottom_right.y, pts.bottom_left.y]
                    cx = (sum(xs)/4) / iw
                    cy = (sum(ys)/4) / ih
                    found.append((r.text, cx, cy))
        except Exception:
            pass
    big = cv2.resize(img, (iw*4, ih*4), interpolation=cv2.INTER_LINEAR)
    bh, bw = big.shape[:2]
    if not found and _wechat is not None:
        try:
            texts, polys = _wechat.detectAndDecode(big)
            for text, poly in zip(texts, (polys or [])):
                if text and poly is not None:
                    cx = float(poly[:, 0].mean()) / bw
                    cy = float(poly[:, 1].mean()) / bh
                    found.append((text, cx, cy))
        except Exception:
            pass
    if not found:
        try:
            val, pts, _ = _detector.detectAndDecode(big)
            if val and pts is not None:
                cx = float(pts[0, :, 0].mean()) / bw
                cy = float(pts[0, :, 1].mean()) / bh
                found.append((val, cx, cy))
        except cv2.error:
            pass
    return found


def _tile_to_world(col, row):
    """Return floor-tile centre in Gazebo world frame (no camera correction).

    Ogre2 box top-face maps texture U→world -Y and texture V→world X (axes swapped
    from intuitive expectation).  The gen_grid_world texture encodes col in U and
    row (inverted via ir=ROWS-1-row) in V, which results in:
        row  → world X  (row=0 westmost, row=ROWS-1 eastmost)
        col  → world -Y (col=0 northmost, col=COLS-1 southmost)
    """
    gz_x = GRID_X0 + row * ROW_SCALE + ROW_SCALE / 2.0
    gz_y = GRID_Y_MAX - (col + 0.5) * COL_SCALE
    return gz_x, gz_y


def _pixel_to_ground_body(norm_u, norm_v):
    """Map normalised image coords (0..1) to body-frame ground offset from base_footprint.

    Camera pitch=pi/2 boresight = straight down.
    Image axes relative to body frame:
      image top  (+V=0) → forward  (+X_body)
      image right (+U=1) → left    (+Y_body)
    Scale: tan(HFOV/2) * height per half-image-width.
    """
    scale_h = _CAM_HEIGHT * math.tan(_CAM_HFOV / 2.0)   # metres per half-width
    scale_v = scale_h * (_IMG_H / _IMG_W)                # metres per half-height
    dx_body = (0.5 - norm_v) * 2.0 * scale_v   # image top → +X_body (forward)
    dy_body = (norm_u - 0.5) * 2.0 * scale_h   # image right → +Y_body (left)
    return _CAM_BODY_X + dx_body, _CAM_BODY_Y + dy_body


def decode_position(bgr, yaw_deg=None):
    """
    Returns (gz_world_x, gz_world_y, confidence) — estimated base_footprint position.

    Uses the QR bounding-box pixel position to project the decoded QR's actual
    ground location, then derives base_footprint from the camera mount geometry.
    yaw_deg: robot heading in degrees (0=east, 90=north, 180=west, -90=south).
    confidence=1.0 on success, 0.0 on failure.
    """
    found = _decode_all(bgr)
    if not found:
        return None, None, 0.0

    # Pick the QR whose bounding-box centre is closest to the image centre.
    best_text, best_u, best_v = min(found, key=lambda r: (r[1]-0.5)**2 + (r[2]-0.5)**2)

    try:
        col, row = map(int, best_text.split(','))
    except ValueError:
        return None, None, 0.0
    if not (0 <= col < COLS and 0 <= row < ROWS):
        return None, None, 0.0

    # World position of the QR tile centre, then correct for pixel offset within tile.
    tile_x, tile_y = _tile_to_world(col, row)
    if yaw_deg is not None:
        bx, by = _pixel_to_ground_body(best_u, best_v)
        yr = math.radians(yaw_deg)
        # ground point in world = base_footprint + R(yaw) * (bx, by)
        # → base_footprint = ground_world - R(yaw) * (bx, by)
        # ground_world ≈ tile_x, tile_y (QR grid identity is ground truth)
        off_x = bx * math.cos(yr) - by * math.sin(yr)
        off_y = bx * math.sin(yr) + by * math.cos(yr)
        return tile_x - off_x, tile_y - off_y, 1.0
    else:
        # No yaw: use nadir offset only (west-heading fallback)
        return tile_x - _CAM_BODY_X, tile_y - _CAM_BODY_Y, 1.0


if __name__ == '__main__':
    import sys
    img = cv2.imread(sys.argv[1]) if len(sys.argv) > 1 else None
    if img is None:
        print("Usage: grid_decoder.py <image.jpg>")
        raise SystemExit(1)
    x, y, conf = decode_position(img)
    print(f"gz_world_x={x}  gz_world_y={y}  confidence={conf}")
