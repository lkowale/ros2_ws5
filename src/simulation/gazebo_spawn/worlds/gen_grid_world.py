#!/usr/bin/env python3
"""
Generate house_short_rows.sdf with a QR-code floor texture for camera localization.

Grid: 26 cols (x) × 24 rows (y), 1m×1m cells, origin at (-4, -18).
A single large PNG texture (grid_floor.png) is applied to one flat plane.
Each cell contains a QR code encoding "col,row" → exact cell identity, no
colour-matching needed, immune to Ogre2 gamma pipeline.

Also writes grid_decoder.py (QR-based) and grid_lut.pkl.
"""

import os, pickle

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

GRID_X0 = -4.0
GRID_Y0 = -18.0   # covers robot path Y: -17 to +5m
CELL    = 0.5      # 0.5m cells → camera covers ~2.8 cells → always ≥1 complete QR
COLS    = 68       # X: -4 to +30m  (34m / 0.5m) — field reaches ~26m east
ROWS    = 48       # Y: -18 to +6m  (24m / 0.5m)
PX      = 64       # texture pixels per cell
QUIET   = 4        # white quiet-zone pixels each side (QR spec min=4 modules)

# ── Texture ───────────────────────────────────────────────────────────────
import cv2, numpy as np

tex_w = COLS * PX
tex_h = ROWS * PX
tex = np.ones((tex_h, tex_w, 3), np.uint8) * 255   # white background

qr_enc = cv2.QRCodeEncoder.create()
QR_PX  = PX - 2 * QUIET   # QR fills cell minus quiet zone margins

for row in range(ROWS):
    for col in range(COLS):
        qr_img = qr_enc.encode(f"{col},{row}")
        qr_resized = cv2.resize(qr_img, (QR_PX, QR_PX), interpolation=cv2.INTER_NEAREST)
        qr_bgr = cv2.cvtColor(qr_resized, cv2.COLOR_GRAY2BGR)
        # Image row 0 = grid row ROWS-1 (OpenGL UV origin at bottom-left)
        ir = ROWS - 1 - row
        y0 = ir * PX + QUIET
        x0 = col * PX + QUIET
        tex[y0:y0+QR_PX, x0:x0+QR_PX] = qr_bgr

tex_path = os.path.join(SCRIPT_DIR, 'grid_floor.png')
cv2.imwrite(tex_path, tex)
print(f"Texture: {tex_w}×{tex_h}px  →  {tex_path}  ({os.path.getsize(tex_path)//1024} KB)")

# ── SDF ───────────────────────────────────────────────────────────────────
floor_w = COLS * CELL
floor_h = ROWS * CELL
cx = GRID_X0 + floor_w / 2.0
cy = GRID_Y0 + floor_h / 2.0

SDF = f"""\
<?xml version='1.0' encoding='ASCII'?>
<sdf version='1.7'>
  <world name='house_short_crop_rows'>
    <physics name="2ms" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"           name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"     name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system"           name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system"               name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system"            name="gz::sim::systems::NavSat"/>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>53.5204991</latitude_deg>
      <longitude_deg>17.8258532</longitude_deg>
      <elevation>100.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.4 0.7 0.9 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>0 0 -1</direction>
    </light>

    <!-- Ground plane: collision only -->
    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
        </collision>
      </link>
    </model>

    <!-- QR-code floor: single textured plane covering the entire grid -->
    <model name='qr_floor'>
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} 0.001 0 0 0</pose>
      <link name='link'>
        <visual name='visual'>
          <geometry>
            <box><size>{floor_w:.3f} {floor_h:.3f} 0.002</size></box>
          </geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0 0 0 1</specular>
            <pbr>
              <metal>
                <albedo_map>file://{tex_path}</albedo_map>
                <metalness>0</metalness>
                <roughness>1</roughness>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>

"""

sdf_path = os.path.join(SCRIPT_DIR, 'house_short_rows.sdf')
with open(sdf_path, 'w') as f:
    f.write(SDF)
    f.write("  </world>\n</sdf>\n")
print(f"Written: {sdf_path}  ({os.path.getsize(sdf_path)//1024} KB)")

# ── Grid decoder ──────────────────────────────────────────────────────────
DECODER = f'''\
#!/usr/bin/env python3
"""
grid_decoder.py — decode robot gz_world position from downward camera image via QR code.

Grid: {COLS} cols x {ROWS} rows, cell={CELL}m, origin=({GRID_X0},{GRID_Y0}).
Each floor cell contains a QR code encoding "col,row".

Ogre2 UV mapping for a box top face swaps the texture axes relative to world XY:
  QR col (texture U) maps to world Y; QR row (texture V) maps to world X.
  The U axis is also inverted: col=0 is world Y_max, col=COLS-1 is world Y_min.

Usage:
    from grid_decoder import decode_position
    gz_world_x, gz_world_y, confidence = decode_position(bgr_image)
"""

import cv2, numpy as np

GRID_X0 = {GRID_X0}
GRID_Y0 = {GRID_Y0}
CELL    = {CELL}
COLS    = {COLS}
ROWS    = {ROWS}

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
    print(f"gz_world_x={{x}}  gz_world_y={{y}}  confidence={{conf}}")
'''

decoder_path = os.path.join(SCRIPT_DIR, 'grid_decoder.py')
with open(decoder_path, 'w') as f:
    f.write(DECODER)
print(f"Written: {decoder_path}")

# ── LUT ───────────────────────────────────────────────────────────────────
cells_lut = [{'col': col, 'row': row,
              'cx': GRID_X0 + col*CELL + CELL/2,
              'cy': GRID_Y0 + row*CELL + CELL/2}
             for row in range(ROWS) for col in range(COLS)]
lut_path = os.path.join(SCRIPT_DIR, 'grid_lut.pkl')
with open(lut_path, 'wb') as f:
    pickle.dump({'cells': cells_lut,
                 'x0': GRID_X0, 'y0': GRID_Y0,
                 'cell': CELL, 'cols': COLS, 'rows': ROWS}, f)
print(f"Written: {lut_path}")
print("Done.")
