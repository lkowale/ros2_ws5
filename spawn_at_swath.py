#!/usr/bin/env python3
"""Compute Gazebo spawn pose (x_pose, y_pose, yaw) for a given swath index.

Converts the swath's start lat/lon (from the field's *_directed_turns.geojson,
same file+ordering field_navigator.cpp loads) into Gazebo world-frame x/y using
the identical linear ENU projection sim_gps_fix_publisher.py uses at runtime,
anchored at the same fixed datum. Since /gps/fix is derived FROM Gazebo world
ground truth about that datum (not the other way around), and map==odom is an
identity transform in sim (see ros_stack_summary memory), this places the robot
at the position the EKF/GPS pipeline will itself compute for that swath start —
not a value read off Gazebo's separate wheel-odom topic (odometry/gazebo),
which drifts and is never fused.

Usage:
    python3 spawn_at_swath.py <field_name> <line_index>
    python3 spawn_at_swath.py house_short 6

Prints x_pose/y_pose/yaw suitable for:
    bash run_m4_field_sim.sh field house_short 6 \\
        --launch-arg x_pose:=<x> y_pose:=<y> yaw:=<yaw>
or directly:
    ros2 launch gazebo_spawn m4_nav_sim.launch.py x_pose:=<x> y_pose:=<y> yaw:=<yaw>
"""
import json
import math
import os
import sys

# Must match sim_gps_fix_publisher.py datum + WGS84 constants exactly.
DATUM_LAT_DEG = 53.5204991
DATUM_LON_DEG = 17.8258532
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2 - _F)

FIELDS_DIR = os.path.join(os.path.expanduser('~'), 'ros2_ws5', 'src', 'fields')


def latlon_to_world_xy(lat_deg, lon_deg):
    lat0 = math.radians(DATUM_LAT_DEG)
    lon0 = math.radians(DATUM_LON_DEG)
    sin_lat = math.sin(lat0)
    denom = math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    m_per_rad_lat = _A * (1.0 - _E2) / (denom ** 3)
    m_per_rad_lon = _A * math.cos(lat0) / denom

    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    x = (lon - lon0) * m_per_rad_lon
    y = (lat - lat0) * m_per_rad_lat
    return x, y


def find_geojson(field_name):
    field_dir = os.path.join(FIELDS_DIR, field_name)
    for fn in os.listdir(field_dir):
        if fn.endswith('_directed_turns.geojson'):
            return os.path.join(field_dir, fn)
    raise FileNotFoundError(f'No *_directed_turns.geojson in {field_dir}')


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)

    field_name = sys.argv[1]
    line_index = int(sys.argv[2])

    geojson_path = find_geojson(field_name)
    with open(geojson_path) as f:
        data = json.load(f)

    features = data['features']
    if not (0 <= line_index < len(features)):
        print(f'line_index {line_index} out of range (0..{len(features) - 1})',
              file=sys.stderr)
        sys.exit(1)

    feat = features[line_index]
    coords = feat['geometry']['coordinates']
    start_lon, start_lat = coords[0]
    end_lon, end_lat = coords[1]

    sx, sy = latlon_to_world_xy(start_lat, start_lon)
    ex, ey = latlon_to_world_xy(end_lat, end_lon)
    yaw = math.atan2(ey - sy, ex - sx)

    print(f'# field={field_name} line_index={line_index} '
          f'src_index={feat["properties"].get("src_index")} '
          f'turn={feat["properties"].get("turn")}')
    print(f'# swath start=({sx:.3f},{sy:.3f}) end=({ex:.3f},{ey:.3f}) '
          f'yaw={math.degrees(yaw):.1f}deg')
    print(f'x_pose:={sx:.4f} y_pose:={sy:.4f} yaw:={yaw:.6f}')


if __name__ == '__main__':
    main()
