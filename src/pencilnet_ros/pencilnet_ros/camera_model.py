"""Camera projection model for the X3 drone.

Projection chain: World -> Body -> Camera Link -> Optical -> Pixels

Camera parameters from model.sdf:
  Mount: position (-0.0015, 0.00604, 0.0623), pitch -0.523599 rad (-30 deg)
  Intrinsics: hfov=1.309 rad (75 deg), image 1280x720
  fx = fy = (width/2) / tan(hfov/2) = ~834.1
  cx = 640, cy = 360

Note: The SDF pitch of -0.523599 rad means the camera looks 30 deg UP
from the body-forward axis. This is typical for racing drones — the camera
tilts up so it points level when the drone is tilted forward during fast
flight. We use the SDF value directly: θ = -0.523599.
"""

import math
import numpy as np


# Camera mount in body frame
CAM_OFFSET = np.array([-0.0015, 0.00604, 0.0623])
CAM_PITCH = -0.523599  # rad (30 deg upward from body-forward, racing drone config)

# Image dimensions
IMG_W = 1280
IMG_H = 720

# Intrinsics: fx = fy = (width/2) / tan(hfov/2)
HFOV = 1.309  # rad
FX = (IMG_W / 2.0) / math.tan(HFOV / 2.0)
FY = FX  # square pixels
CX = IMG_W / 2.0
CY = IMG_H / 2.0


def _rotation_matrix_z(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1],
    ])


def _rotation_matrix_y(pitch):
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ])


def _rotation_matrix_x(roll):
    c, s = math.cos(roll), math.sin(roll)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c],
    ])


def euler_to_rotation(roll, pitch, yaw):
    """ZYX Euler angles to rotation matrix (body-to-world)."""
    return _rotation_matrix_z(yaw) @ _rotation_matrix_y(pitch) @ _rotation_matrix_x(roll)


def quat_to_yaw(qx, qy, qz, qw):
    """Extract yaw from quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_to_rotation(qx, qy, qz, qw):
    """Quaternion (x,y,z,w) to rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


# Pre-compute camera mount rotation (pitch only, no roll/yaw)
_R_cam_mount = euler_to_rotation(0.0, CAM_PITCH, 0.0)

# Rotation from camera link frame to optical frame
# optical: x=right, y=down, z=forward (from cam link: x=forward, y=left, z=up)
_R_optical = np.array([
    [0, -1,  0],
    [0,  0, -1],
    [1,  0,  0],
], dtype=float)


def project_points_to_pixels(points_world, drone_pos, drone_quat):
    """Project 3D world points to pixel coordinates.

    Args:
        points_world: (N, 3) array of 3D points in world frame.
        drone_pos: (3,) drone position in world frame (x, y, z).
        drone_quat: (4,) drone orientation as (qx, qy, qz, qw).

    Returns:
        pixels: (N, 2) array of (u, v) pixel coordinates.
        valid: (N,) boolean mask — True if point is in front of camera
               and within image bounds.
    """
    points_world = np.asarray(points_world, dtype=float)
    drone_pos = np.asarray(drone_pos, dtype=float)

    qx, qy, qz, qw = drone_quat
    R_body = quat_to_rotation(qx, qy, qz, qw)

    # Step 1: World -> Body frame
    points_body = (R_body.T @ (points_world - drone_pos).T).T

    # Step 2: Body -> Camera link frame
    points_cam_link = (_R_cam_mount.T @ (points_body - CAM_OFFSET).T).T

    # Step 3: Camera link -> Optical frame
    points_optical = (_R_optical @ points_cam_link.T).T

    # Step 4: Optical -> Pixels (pinhole projection)
    z = points_optical[:, 2]
    valid_z = z > 0.1  # in front of camera

    # Avoid divide by zero
    z_safe = np.where(valid_z, z, 1.0)
    u = FX * (points_optical[:, 0] / z_safe) + CX
    v = FY * (points_optical[:, 1] / z_safe) + CY

    # Check bounds
    valid_bounds = (u >= 0) & (u < IMG_W) & (v >= 0) & (v < IMG_H)
    valid = valid_z & valid_bounds

    pixels = np.stack([u, v], axis=-1)
    return pixels, valid


def project_gate_to_image(gate_corners_world, drone_pos, drone_quat):
    """Project gate corners to image and compute centroid from projected corners.

    Args:
        gate_corners_world: List of (x, y, z) gate corners in world frame.
        drone_pos: (3,) drone position.
        drone_quat: (4,) drone quaternion (qx, qy, qz, qw).

    Returns:
        dict with 'visible', 'center_x', 'center_y' (centroid of projected
        corners, clipped to image), 'corners_px' (N,2 projected corners),
        'valid_mask', or None if gate is not visible.
    """
    corners = np.array(gate_corners_world)
    pixels, valid = project_points_to_pixels(corners, drone_pos, drone_quat)

    if not np.any(valid):
        return None

    # Compute center from centroid of ALL corners in front of camera,
    # clipped to image bounds.  This is more accurate than an axis-aligned
    # bounding-box center when the gate is viewed at an angle.
    points_world = np.asarray(gate_corners_world, dtype=float)
    drone_pos_arr = np.asarray(drone_pos, dtype=float)
    qx, qy, qz, qw = drone_quat
    R_body = quat_to_rotation(qx, qy, qz, qw)
    points_body = (R_body.T @ (points_world - drone_pos_arr).T).T
    points_cam = (_R_cam_mount.T @ (points_body - CAM_OFFSET).T).T
    points_opt = (_R_optical @ points_cam.T).T
    in_front = points_opt[:, 2] > 0.1

    if not np.any(in_front):
        return None

    front_pixels = pixels[in_front]

    # Centroid of projected corners (unclipped)
    center_x = float(front_pixels[:, 0].mean())
    center_y = float(front_pixels[:, 1].mean())

    # Reject if centroid is outside image bounds
    if center_x < 0 or center_x >= IMG_W or center_y < 0 or center_y >= IMG_H:
        return None

    return {
        'visible': True,
        'center_x': center_x,
        'center_y': center_y,
        'corners_px': pixels,
        'valid_mask': valid,
        'in_front_mask': in_front,
    }


def compute_gate_distance(gate_pos, drone_pos):
    """Euclidean distance from drone to gate center."""
    gp = np.asarray(gate_pos, dtype=float)
    dp = np.asarray(drone_pos, dtype=float)
    return float(np.linalg.norm(gp - dp))


def compute_gate_z_depth(gate_pos, drone_pos, drone_quat):
    """Z-depth of gate center in camera optical frame.

    This matches what Gazebo's depth camera returns (distance along
    optical axis), allowing tight tolerance comparison with the depth
    buffer without the Euclidean-vs-Z mismatch.
    """
    gp = np.asarray(gate_pos, dtype=float).reshape(1, 3)
    dp = np.asarray(drone_pos, dtype=float)
    qx, qy, qz, qw = drone_quat
    R_body = quat_to_rotation(qx, qy, qz, qw)
    p_body = (R_body.T @ (gp - dp).T).T
    p_cam = (_R_cam_mount.T @ (p_body - CAM_OFFSET).T).T
    p_opt = (_R_optical @ p_cam.T).T
    return float(p_opt[0, 2])


def compute_relative_yaw(gate_yaw, drone_yaw):
    """Relative yaw from drone heading to gate normal, wrapped to [-pi, pi]."""
    diff = gate_yaw - drone_yaw
    return (diff + math.pi) % (2 * math.pi) - math.pi
