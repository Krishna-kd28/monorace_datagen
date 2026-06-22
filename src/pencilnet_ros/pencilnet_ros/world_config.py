"""World configurations for gate positions and auto-detection."""

import subprocess
import math

# (name, x, y, yaw_rad, is_double)
WORLDS = {
    "x3_a2rl_warehouse": {
        "start": (0, 0, 0.7854),
        "gates": [
            ("gate1",   2.0,    0.0,   0.0,       False),
            ("gate2",   8.0,   -4.0,  -0.7854,    False),
            ("gate3",   9.0,  -12.0,  -1.0472,    False),
            ("gate4",  12.0,  -22.0,  -1.5708,    False),
            ("gate5",   7.0,  -28.0,  -0.17454,   False),
            ("gate6",   2.0,  -20.0,   0.0,       False),
            ("gate7",  -3.0,  -28.0,  -1.74534,   True),
            ("gate8",  -4.0,  -20.0,  -1.396267,  False),
            ("gate9",  -6.0,  -12.0,  -1.74534,   False),
            ("gate10", -4.0,   -4.0,  -2.3562,    True),
        ],
    },
    "x3_a2rl_warehouse_harmonic": {
        "start": (12, 34, 0.7854),
        "gates": [
            ("gate1",  14.0,   34.0,   0.0,       False),
            ("gate2",  20.0,   30.0,  -0.7854,    False),
            ("gate3",  21.0,   22.0,  -1.0472,    False),
            ("gate4",  24.0,   12.0,  -1.5708,    False),
            ("gate5",  19.0,    6.0,  -0.17454,   False),
            ("gate6",  14.0,   14.0,   0.0,       False),
            ("gate7",   9.0,    6.0,  -1.74534,   True),
            ("gate8",   8.0,   14.0,  -1.396267,  False),
            ("gate9",   6.0,   22.0,  -1.74534,   False),
            ("gate10",  8.0,   30.0,  -2.3562,    True),
        ],
    },
}

# Gate geometry in gate-local frame (gate center at origin, facing +X)
# Dimensions from COLLADA mesh analysis:
#   Transform matrix: scale_x=0.07, scale_y=1.35, scale_z=1.35, translate_z=1.45
#   Unit cube vertices at ±1.0 map to ±1.35m (Y), ±1.35+1.45 (Z)
#   Using outer frame corners — text panels between inner/outer edges are
#   prominent in pencil filter and define what the network should detect.

# Single gate: outer frame 2.7m wide x 2.7m tall, z from 0.10 to 2.80
SINGLE_GATE_CORNERS = [
    (0.0, -1.350, 0.100),   # bottom-left
    (0.0,  1.350, 0.100),   # bottom-right
    (0.0,  1.350, 2.800),   # top-right
    (0.0, -1.350, 2.800),   # top-left
]

# Double gate: outer frame 2.7m wide x 5.4m tall, z from 0.10 to 5.50
DOUBLE_GATE_CORNERS = [
    (0.0, -1.350, 0.100),   # bottom-left
    (0.0,  1.350, 0.100),   # bottom-right
    (0.0,  1.350, 5.500),   # top-right
    (0.0, -1.350, 5.500),   # top-left
]

FLIGHT_HEIGHT = 1.45         # meters — single gate inner opening center
DOUBLE_GATE_HEIGHT = 2.8     # meters — double gate center
APPROACH_DIST = 3.0
EXIT_DIST = 2.0


def detect_world():
    """Auto-detect which world is running by checking Gazebo services."""
    try:
        result = subprocess.run(
            ["gz", "service", "-l"],
            capture_output=True, text=True, timeout=3,
        )
        for name in WORLDS:
            if f"/world/{name}/set_pose" in result.stdout:
                return name
    except Exception:
        pass
    return None


def get_gate_corners_world(gx, gy, gyaw, is_double):
    """Transform gate corners from gate-local frame to world frame.

    Args:
        gx, gy: Gate position in world frame.
        gyaw: Gate yaw angle in world frame.
        is_double: True for double gate (4m tall), False for single (2m tall).

    Returns:
        List of (x, y, z) tuples in world frame.
    """
    corners_local = DOUBLE_GATE_CORNERS if is_double else SINGLE_GATE_CORNERS
    cos_y = math.cos(gyaw)
    sin_y = math.sin(gyaw)

    corners_world = []
    for lx, ly, lz in corners_local:
        wx = gx + cos_y * lx - sin_y * ly
        wy = gy + sin_y * lx + cos_y * ly
        wz = lz
        corners_world.append((wx, wy, wz))
    return corners_world
