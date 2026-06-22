#!/usr/bin/env python3
"""
Teleport-based data generator using triggered cameras + depth visibility.

Architecture:
  - Uses camera_rig model (no control plugins) + physics disabled in
    Gazebo, so set_pose + step(1) renders with zero drift.
  - TRIGGERED cameras: one Bool publish fires both RGB + depth on the
    same update cycle (deterministic, no multi-step guessing).
  - Ground-truth pose: uses the TELEPORTED viewpoint pose (not odom),
    eliminating all timing/threading race conditions.
  - Depth-based occlusion: gates whose properly-clipped polygon has <15%
    depth agreement with expected Z-depth are dropped.
  - Double-sided gates: oblique angle filter uses abs() (gates look the
    same from front and back).  yaw_relative is normalised to the
    visible face.

Capture sequence per viewpoint:
  set_pose → trigger → step(1) → wait RGB+depth → annotate

PREREQUISITE — triggered cameras (already configured in this package):
  This node fires /camera_trigger via `gz topic` per viewpoint and waits
  for the next RGB + depth frame. The camera_rig model used by the
  data-generation world (mav_description/models/camera_rig/model.sdf) ships
  with `<triggered>true</triggered>` + `<trigger_topic>/camera_trigger</...>`
  already enabled on both sensors, so no SDF edits are needed — just build
  and launch (see README.md).

Usage:
    ros2 run pencilnet_ros data_generator --ros-args \
        -p output_dir:=/home/user/pencilnet_dataset
"""

import math
import os
import json
import time
import threading
import subprocess

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from pencilnet_ros.world_config import (
    WORLDS, detect_world, get_gate_corners_world,
    FLIGHT_HEIGHT, DOUBLE_GATE_HEIGHT,
)
from pencilnet_ros.camera_model import (
    project_gate_to_image, compute_gate_distance,
    compute_gate_z_depth, compute_relative_yaw, IMG_W, IMG_H,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _euler_to_quat(yaw, pitch, roll=0.0):
    """ZYX Euler → quaternion (qx, qy, qz, qw)."""
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def _decode_depth(msg):
    """ROS Image → float32 meters HxW, or None."""
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width).copy()
    if msg.encoding == '16UC1':
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(
            msg.height, msg.width).astype(np.float32)
        return d / 1000.0
    return None


def _bbox_intersection(xmin, ymin, xmax, ymax, W, H):
    """Intersection of an unclipped bbox with the image [0..W-1, 0..H-1].

    Returns (inter_area, inter_ratio, (ixmin, iymin, ixmax, iymax)).
    inter_ratio = intersection / original bbox area.
    """
    ixmin = max(0.0, xmin)
    iymin = max(0.0, ymin)
    ixmax = min(float(W - 1), xmax)
    iymax = min(float(H - 1), ymax)
    iw = max(0.0, ixmax - ixmin)
    ih = max(0.0, iymax - iymin)
    inter = iw * ih
    area = max(1.0, (xmax - xmin) * (ymax - ymin))
    return inter, inter / area, (ixmin, iymin, ixmax, iymax)


def _clip_poly_to_rect(poly, W, H):
    """Sutherland-Hodgman clip polygon to [0..W-1, 0..H-1]."""
    def clip_edge(points, inside, intersect):
        out = []
        if not points:
            return out
        prev = points[-1]
        prev_in = inside(prev)
        for cur in points:
            cur_in = inside(cur)
            if cur_in:
                if not prev_in:
                    out.append(intersect(prev, cur))
                out.append(cur)
            elif prev_in:
                out.append(intersect(prev, cur))
            prev, prev_in = cur, cur_in
        return out

    x0, y0 = 0.0, 0.0
    x1, y1 = float(W - 1), float(H - 1)
    pts = [tuple(map(float, p)) for p in poly]

    # left
    pts = clip_edge(
        pts,
        inside=lambda p: p[0] >= x0,
        intersect=lambda a, b: (
            x0, a[1] + (b[1] - a[1]) * (x0 - a[0]) / (b[0] - a[0] + 1e-9)))
    # right
    pts = clip_edge(
        pts,
        inside=lambda p: p[0] <= x1,
        intersect=lambda a, b: (
            x1, a[1] + (b[1] - a[1]) * (x1 - a[0]) / (b[0] - a[0] + 1e-9)))
    # top
    pts = clip_edge(
        pts,
        inside=lambda p: p[1] >= y0,
        intersect=lambda a, b: (
            a[0] + (b[0] - a[0]) * (y0 - a[1]) / (b[1] - a[1] + 1e-9), y0))
    # bottom
    pts = clip_edge(
        pts,
        inside=lambda p: p[1] <= y1,
        intersect=lambda a, b: (
            a[0] + (b[0] - a[0]) * (y1 - a[1]) / (b[1] - a[1] + 1e-9), y1))
    return pts


def _visibility_ratio(depth_m, poly_px, z_depth_min, z_depth_max,
                      rel_tol=0.15, max_tol=3.0, min_px=40):
    """Occlusion-aware visibility check using depth.

    Classifies polygon pixels into three buckets:
      matching  – depth within [z_depth_min - tol, z_depth_max + tol]
                  (gate frame visible — range covers oblique viewing angles)
      occluding – depth < z_depth_min - tol (something closer blocks gate)
      opening   – depth > z_depth_max + tol or infinite (seeing through gate)

    Returns matching / (matching + occluding).  Opening pixels are ignored
    so close gates (whose opening shows far-away backgrounds) are not
    penalised, while gates half-hidden behind a closer gate's frame are
    correctly rejected.

    z_depth_min/max are the Z-depths of the nearest/farthest gate corners
    in the camera optical frame.  At oblique angles the gate frame spans a
    wide depth range; using both ends prevents the near edge from being
    misclassified as occluding.
    """
    H, W = depth_m.shape
    pts = np.asarray(poly_px, dtype=np.float32)
    if pts.shape[0] < 3:
        return 0.0

    # convexHull fixes ordering after Sutherland-Hodgman clipping
    hull = cv2.convexHull(pts.reshape(-1, 1, 2))
    mask = np.zeros((H, W), dtype=np.uint8)
    cv2.fillConvexPoly(mask, hull.astype(np.int32), 1)
    # Erode 1px to avoid noisy depth edges
    mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=1)

    d = depth_m[mask.astype(bool)]
    d = d[np.isfinite(d) & (d > 0.05)]
    if d.size < min_px:
        return 0.0

    tol_lo = min(rel_tol * z_depth_min, max_tol)
    tol_hi = min(rel_tol * z_depth_max, max_tol)
    matching = int(np.sum((d >= z_depth_min - tol_lo) &
                          (d <= z_depth_max + tol_hi)))
    occluding = int(np.sum(d < z_depth_min - tol_lo))

    denom = matching + occluding
    if denom < min_px:
        return 0.0

    return float(matching) / float(denom)


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class DataGeneratorNode(Node):
    def __init__(self):
        super().__init__('data_generator')

        self.declare_parameter('output_dir', os.path.expanduser(
            '~/pencilnet_dataset'))
        self.declare_parameter('trigger_topic', '/camera_trigger')
        self.declare_parameter('closest_gate_only', False)
        self.declare_parameter('num_viewpoints', 15000)
        # Log depth encoding on first message for debugging
        self._depth_encoding_logged = False
        self.output_dir = self.get_parameter('output_dir').value
        self.trigger_topic = self.get_parameter('trigger_topic').value
        self.closest_gate_only = self.get_parameter(
            'closest_gate_only').value
        self.num_viewpoints = self.get_parameter('num_viewpoints').value

        world_name = detect_world()
        if world_name is None:
            self.get_logger().error(
                "No supported world detected. Is the simulator running?")
            raise RuntimeError("No world detected")

        self.world_name = world_name
        self.world_cfg = WORLDS[world_name]
        self.gates = self.world_cfg["gates"]
        self.get_logger().info(
            f"World: {world_name}, {len(self.gates)} gates")

        # Pre-compute gate corners in world frame
        self.gate_corners_world = []
        for name, gx, gy, gyaw, is_double in self.gates:
            corners = get_gate_corners_world(gx, gy, gyaw, is_double)
            gate_center_z = (DOUBLE_GATE_HEIGHT if is_double
                             else FLIGHT_HEIGHT)
            self.gate_corners_world.append({
                'name': name, 'corners': corners,
                'gx': gx, 'gy': gy, 'gyaw': gyaw,
                'center_z': gate_center_z, 'is_double': is_double,
            })

        # ROS subscriptions — RGB + depth
        self.image_sub = self.create_subscription(
            Image, '/X3/camera/image_raw', self._image_cb, 10)
        self.depth_sub = self.create_subscription(
            Image, '/X3/camera/depth_image', self._depth_cb, 10)

        # Thread-safe state
        self._lock = threading.Lock()
        self.latest_frame = None
        self.latest_depth = None
        self.rgb_seq = 0
        self.depth_seq = 0

        # Data collection
        self.annotations_list = []
        self.frame_count = 0

        self.images_dir = os.path.join(self.output_dir, 'images')
        os.makedirs(self.images_dir, exist_ok=True)
        self.get_logger().info(f"Output dir: {self.output_dir}")

        self.viewpoints = self._generate_viewpoints()
        self.get_logger().info(
            f"Generated {len(self.viewpoints)} viewpoints to capture")

        self.running = True
        self.capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self.capture_thread.start()

    # ---- Viewpoint generation ----

    def _generate_viewpoints(self):
        """Generate randomized viewpoints around each gate.

        Continuous sampling with jitter on all axes for maximum diversity.
        Produces self.num_viewpoints total, distributed evenly across
        gates and approach sides.
        """
        rng = np.random.default_rng(seed=42)
        viewpoints = []
        n_gates = len(self.gates)
        # Split evenly: each gate gets samples from both sides
        per_gate_side = max(1, self.num_viewpoints // (n_gates * 2))

        for gate_idx, (name, gx, gy, gyaw, is_double) in enumerate(
                self.gates):
            nominal_h = (DOUBLE_GATE_HEIGHT if is_double else FLIGHT_HEIGHT)

            for side in [0, math.pi]:
                base_angle = gyaw + side

                for _ in range(per_gate_side):
                    # Distance: log-uniform [2.5, 18] — more close-up samples
                    dist = float(np.exp(rng.uniform(
                        np.log(2.5), np.log(18.0))))

                    # Angle offset: uniform ±40° around gate normal
                    angle_off = rng.uniform(-0.70, 0.70)
                    approach_angle = base_angle + angle_off

                    # Lateral offset: perpendicular to approach direction
                    lateral = rng.uniform(-1.5, 1.5)
                    perp_angle = approach_angle + math.pi / 2.0

                    dx = -math.cos(approach_angle) * dist
                    dy = -math.sin(approach_angle) * dist
                    dx += math.cos(perp_angle) * lateral
                    dy += math.sin(perp_angle) * lateral

                    vx, vy = gx + dx, gy + dy

                    # Height: nominal ± jitter
                    h_jitter = rng.uniform(-0.5, 0.5)
                    vz = nominal_h + h_jitter
                    if vz < 0.5:
                        vz = 0.5

                    # Yaw: face gate center + jitter ±15°
                    face_yaw = math.atan2(gy - vy, gx - vx)
                    yaw_jitter = rng.uniform(-0.26, 0.26)

                    # Pitch: base range [0.35, 0.55] + jitter ±7°
                    pitch = rng.uniform(0.23, 0.67)

                    # Roll: small ±3°
                    roll = rng.uniform(-0.052, 0.052)

                    viewpoints.append({
                        'x': vx, 'y': vy, 'z': vz,
                        'yaw': face_yaw + yaw_jitter,
                        'pitch': pitch, 'roll': roll,
                        'target_gate': name,
                        'gate_idx': gate_idx,
                    })

        rng.shuffle(viewpoints)
        return viewpoints

    # ---- Gazebo subprocess helpers ----

    def _gz_service(self, service, reqtype, reptype, req_str):
        """Call gz service via subprocess.  Returns True on success."""
        try:
            r = subprocess.run(
                ["gz", "service",
                 "-s", f"/world/{self.world_name}/{service}",
                 "--reqtype", reqtype, "--reptype", reptype,
                 "--timeout", "2000", "--req", req_str],
                capture_output=True, timeout=5, text=True)
            if r.returncode != 0:
                self.get_logger().warn(
                    f"gz {service} rc={r.returncode}: {r.stderr.strip()}")
                return False
            return True
        except Exception as e:
            self.get_logger().warn(f"gz {service} failed: {e}")
            return False

    def _gz_set_pose(self, x, y, z, yaw=0.0, pitch=0.0, roll=0.0):
        qx, qy, qz, qw = _euler_to_quat(yaw, pitch, roll)
        req = (f"name: 'X3', position: {{x: {x}, y: {y}, z: {z}}}, "
               f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}")
        return self._gz_service(
            'set_pose', 'gz.msgs.Pose', 'gz.msgs.Boolean', req)

    def _gz_step(self, n=1):
        return self._gz_service('control', 'gz.msgs.WorldControl',
                                'gz.msgs.Boolean', f'multi_step: {n}')

    def _gz_pause(self, pause=True):
        val = 'true' if pause else 'false'
        return self._gz_service('control', 'gz.msgs.WorldControl',
                                'gz.msgs.Boolean', f'pause: {val}')

    def _gz_trigger(self):
        """Publish trigger — fires both RGB and depth cameras."""
        try:
            subprocess.run(
                ["gz", "topic", "-t", self.trigger_topic,
                 "-m", "gz.msgs.Boolean", "-p", "data: true", "-n", "1"],
                capture_output=True, timeout=3)
        except Exception:
            pass

    # ---- ROS callbacks (main thread via rclpy.spin) ----

    def _image_cb(self, msg):
        if msg.encoding == 'rgb8':
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        else:
            return
        with self._lock:
            self.latest_frame = frame
            self.rgb_seq += 1

    def _depth_cb(self, msg):
        if not self._depth_encoding_logged:
            self.get_logger().info(f"Depth encoding: {msg.encoding}")
            self._depth_encoding_logged = True
        depth = _decode_depth(msg)
        if depth is not None:
            with self._lock:
                self.latest_depth = depth
                self.depth_seq += 1

    # ---- Main capture loop (separate thread) ----

    def _capture_loop(self):
        self.get_logger().info("Triggering initial camera capture...")
        # Triggered cameras need an explicit trigger + step to publish
        self._gz_trigger()
        self._gz_step(1)

        deadline = time.time() + 10.0
        while self.running and time.time() < deadline:
            with self._lock:
                have_both = (self.latest_frame is not None
                             and self.latest_depth is not None)
            if have_both:
                break
            time.sleep(0.1)

        with self._lock:
            have_both = (self.latest_frame is not None
                         and self.latest_depth is not None)
        if not have_both:
            self.get_logger().error(
                "Timeout waiting for cameras.  "
                f"RGB={'OK' if self.latest_frame is not None else 'MISSING'}, "
                f"Depth={'OK' if self.latest_depth is not None else 'MISSING'}")
            return

        self.get_logger().info(
            "Both cameras ready.  Physics disabled (camera_rig mode).")
        self._gz_pause(True)
        time.sleep(0.3)

        start_time = time.time()
        skipped = 0

        for i, vp in enumerate(self.viewpoints):
            if not self.running:
                break

            # 1. Teleport (static model, no physics → stays exactly here)
            self._gz_set_pose(
                vp['x'], vp['y'], vp['z'], vp['yaw'], vp['pitch'],
                vp.get('roll', 0.0))

            # 2. Record pre-step sequence numbers
            with self._lock:
                rgb_before = self.rgb_seq
                depth_before = self.depth_seq

            # 3. Trigger cameras then step (sensor renders during step)
            self._gz_trigger()
            self._gz_step(1)

            # 4. Wait for BOTH new RGB and depth (edge-triggered)
            dl = time.time() + 3.0
            got_rgb = got_depth = False
            while time.time() < dl:
                with self._lock:
                    got_rgb = self.rgb_seq > rgb_before
                    got_depth = self.depth_seq > depth_before
                if got_rgb and got_depth:
                    break
                time.sleep(0.02)

            if got_rgb and got_depth:
                with self._lock:
                    frame = self.latest_frame.copy()
                    depth = self.latest_depth.copy()
                if i < 5:
                    self.get_logger().info(
                        f"VP {i}: got frame+depth, "
                        f"depth range [{np.nanmin(depth):.2f}, "
                        f"{np.nanmax(depth):.2f}]m")
                self._capture_and_annotate(frame, depth, vp)
            else:
                skipped += 1
                if i < 5:
                    self.get_logger().warn(
                        f"VP {i}: timeout rgb={got_rgb} "
                        f"depth={got_depth}")

            # Progress every 100 viewpoints
            if (i + 1) % 100 == 0:
                elapsed = time.time() - start_time
                rate = (i + 1) / elapsed
                eta = (len(self.viewpoints) - i - 1) / rate if rate > 0 else 0
                self.get_logger().info(
                    f"VP {i+1}/{len(self.viewpoints)}, "
                    f"{self.frame_count} saved, {skipped} skip, "
                    f"{rate:.1f} vp/s, ETA {eta/60:.1f}min")
                self._save_annotations()

        self._gz_pause(False)
        self._save_dataset()
        self.get_logger().info(
            f"DONE! {self.frame_count} frames from "
            f"{len(self.viewpoints)} viewpoints ({skipped} skipped)")

    # ---- Annotation ----

    def _capture_and_annotate(self, frame, depth, vp):
        # Ground truth from teleported pose (static model, no physics drift)
        drone_pos = np.array([vp['x'], vp['y'], vp['z']])
        drone_quat = _euler_to_quat(vp['yaw'], vp['pitch'],
                                     vp.get('roll', 0.0))
        drone_yaw = vp['yaw']

        candidates = []
        for gate_info in self.gate_corners_world:
            gx, gy = gate_info['gx'], gate_info['gy']

            # Gate → drone vector
            g2d_x = drone_pos[0] - gx
            g2d_y = drone_pos[1] - gy
            dist_horiz = math.sqrt(g2d_x**2 + g2d_y**2)

            # Oblique angle handling (two thresholds):
            #   abs(dot) < 0.05 (~87°): hard reject (edge-on sliver)
            #   0.05 <= abs(dot) < 0.30 (~72°): presence-only (p=1,
            #       but regression targets masked in training loss)
            #   abs(dot) >= 0.30: full label (presence + regression)
            if dist_horiz > 0.1:
                gnx = math.cos(gate_info['gyaw'])
                gny = math.sin(gate_info['gyaw'])
                dot = gnx * (g2d_x / dist_horiz) + gny * (g2d_y / dist_horiz)
                dot_abs = abs(dot)
                if dot_abs < 0.05:
                    continue  # edge-on, too ambiguous
                good_view = (dot_abs >= 0.30)
                visible_face_yaw = (gate_info['gyaw'] if dot > 0
                                    else gate_info['gyaw'] + math.pi)
            else:
                visible_face_yaw = gate_info['gyaw']
                good_view = True

            result = project_gate_to_image(
                gate_info['corners'], drone_pos, drone_quat)
            if result is None:
                continue

            cpx = result['corners_px']
            in_front = result.get('in_front_mask', result['valid_mask'])
            front_px = cpx[in_front]
            if len(front_px) < 2:
                continue

            # --- FRUSTUM CHECK on raw (unclipped) projected points ---
            xmin_u = float(front_px[:, 0].min())
            xmax_u = float(front_px[:, 0].max())
            ymin_u = float(front_px[:, 1].min())
            ymax_u = float(front_px[:, 1].max())

            inter_area, inter_ratio, (ixmin, iymin, ixmax, iymax) = \
                _bbox_intersection(xmin_u, ymin_u, xmax_u, ymax_u,
                                   IMG_W, IMG_H)

            # Reject completely out-of-frame gates
            if inter_area <= 0:
                continue

            # Reject barely-touching-frame (tiny sliver on edge)
            # but let large close gates through via absolute pixel check
            if inter_ratio < 0.01 and inter_area < 400:
                continue

            # Spread check on intersection bbox
            spread = max(ixmax - ixmin, iymax - iymin)
            if spread < 8:
                continue

            # --- CLIP polygon to image rect for depth check ---
            clipped_poly = _clip_poly_to_rect(front_px.tolist(),
                                              IMG_W, IMG_H)
            if len(clipped_poly) < 3:
                continue

            clipped_np = np.array(clipped_poly, dtype=np.float32)
            # convexHull fixes vertex ordering after S-H clipping
            clipped_np = cv2.convexHull(
                clipped_np.reshape(-1, 1, 2)).squeeze(1)

            gate_center_3d = np.array([gx, gy, gate_info['center_z']])
            distance = compute_gate_distance(gate_center_3d, drone_pos)
            z_depth = compute_gate_z_depth(gate_center_3d, drone_pos,
                                           drone_quat)
            yaw_rel = compute_relative_yaw(visible_face_yaw, drone_yaw)

            if z_depth < 0.5:
                continue  # gate center behind camera

            # Z-depth range across all gate corners (handles oblique angles
            # where the near/far edges of the frame differ significantly)
            corner_z_depths = [
                compute_gate_z_depth(c, drone_pos, drone_quat)
                for c in gate_info['corners']
            ]
            corner_z_front = [zd for zd in corner_z_depths if zd > 0.1]
            if not corner_z_front:
                continue
            z_depth_min = min(corner_z_front)
            z_depth_max = max(corner_z_front)

            # Depth visibility check on properly-clipped polygon
            vis = _visibility_ratio(depth, clipped_np,
                                    z_depth_min, z_depth_max)
            if self.frame_count < 3:
                self.get_logger().info(
                    f"  gate {gate_info['name']}: dist={distance:.1f}m "
                    f"z={z_depth:.1f}m vis={vis:.2f} spread={spread:.0f}")
            if vis < 0.25:
                continue

            # Bbox from intersection (guaranteed in-frame)
            xmin = int(math.floor(ixmin))
            ymin = int(math.floor(iymin))
            xmax = int(math.ceil(ixmax))
            ymax = int(math.ceil(iymax))

            # Center from unclipped projected centroid (camera_model
            # already computes this from all in-front corners)
            center_x = result['center_x']
            center_y = result['center_y']
            # Clamp center into image for annotation validity
            center_x = max(0.0, min(center_x, IMG_W - 1.0))
            center_y = max(0.0, min(center_y, IMG_H - 1.0))

            # Store clipped polygon as corners_px for visualization
            candidates.append({
                'center_x': center_x,
                'center_y': center_y,
                'distance': float(distance),
                'yaw_relative': float(yaw_rel),
                'presence_only': not good_view,
                'xmin': xmin, 'ymin': ymin,
                'xmax': xmax, 'ymax': ymax,
                'corners_px': clipped_np.tolist(),
            })

        if len(candidates) == 0:
            return

        # Optionally keep only the closest gate
        if self.closest_gate_only:
            candidates.sort(key=lambda c: c['distance'])
            candidates = candidates[:1]

        img_name = f"frame_{self.frame_count:06d}.jpg"
        cv2.imwrite(os.path.join(self.images_dir, img_name), frame)

        self.annotations_list.append({
            'image': img_name,
            'annotations': candidates,
        })
        self.frame_count += 1

    # ---- Dataset saving ----

    def _save_annotations(self):
        """Periodic save: annotations JSON only (no split recomputation)."""
        path = os.path.join(self.output_dir, 'annotations.json')
        with open(path, 'w') as f:
            json.dump({'annotations': self.annotations_list}, f, indent=2)
        self.get_logger().info(
            f"Annotations saved: {len(self.annotations_list)} frames")

    def _save_dataset(self):
        """Final save: annotations + stable train/test split (once)."""
        self._save_annotations()
        n = len(self.annotations_list)
        if n == 0:
            return
        indices = np.arange(n)
        np.random.seed(42)
        np.random.shuffle(indices)
        split = int(0.8 * n)
        np.save(os.path.join(self.output_dir, 'train-indices.npy'),
                indices[:split])
        np.save(os.path.join(self.output_dir, 'test-indices.npy'),
                indices[split:])
        self.get_logger().info(
            f"Dataset: {n} frames ({split} train, {n - split} test)")


def main(args=None):
    import signal

    rclpy.init(args=args)
    try:
        node = DataGeneratorNode()
    except RuntimeError:
        rclpy.shutdown()
        return

    def _shutdown(signum=None, frame=None):
        node.running = False
        time.sleep(0.5)
        node._gz_pause(False)
        node._save_dataset()
        node.get_logger().info(
            f"Interrupted. Saved {node.frame_count} frames")
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGTERM, _shutdown)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        _shutdown()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
