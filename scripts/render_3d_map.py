#!/usr/bin/env python3
"""
Render 3D map building animation from ORB-SLAM3 output files.
Generates a video showing the 3D point cloud growing over time, like the Pangolin viewer.

Input:  MapPoints.txt + KeyFrameTrajectory.txt (or CameraTrajectory.txt)
Output: 3d_map_animation.mp4

Usage:
    python scripts/render_3d_map.py <result_dir>/orbslam3/
    python scripts/render_3d_map.py <result_dir>/orbslam3/ --fps 30 --width 1024 --height 768
"""

import numpy as np
import cv2
import os
import sys
import argparse


def load_map_points(filepath):
    """Load 3D points from MapPoints.txt (x y z per line)."""
    points = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 3:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(points) if points else np.zeros((0, 3))


def load_trajectory(filepath):
    """Load trajectory from TUM format: timestamp tx ty tz qx qy qz qw."""
    poses = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                ts = float(parts[0])
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                poses.append((ts, tx, ty, tz))
    return poses


def project_3d_to_2d(points_3d, cam_center, width, height, scale, offset_x, offset_y):
    """Simple orthographic projection of 3D points to 2D (top-down XZ view)."""
    if len(points_3d) == 0:
        return np.zeros((0, 2), dtype=int)
    # Use X and Z coordinates (top-down view)
    pts_2d = np.zeros((len(points_3d), 2), dtype=int)
    pts_2d[:, 0] = ((points_3d[:, 0] - cam_center[0]) * scale + width / 2 + offset_x).astype(int)
    pts_2d[:, 1] = ((points_3d[:, 2] - cam_center[2]) * scale + height / 2 + offset_y).astype(int)
    return pts_2d


def render_3d_animation(slam_dir, fps=30, width=1024, height=768, num_frames=300):
    """Render 3D map building animation."""

    # Load data
    map_file = os.path.join(slam_dir, "MapPoints.txt")
    traj_file = os.path.join(slam_dir, "KeyFrameTrajectory.txt")
    if not os.path.exists(traj_file):
        traj_file = os.path.join(slam_dir, "CameraTrajectory.txt")

    if not os.path.exists(map_file):
        print(f"Error: {map_file} not found")
        return None
    if not os.path.exists(traj_file):
        print(f"Error: No trajectory file found in {slam_dir}")
        return None

    print(f"Loading map points from {map_file}...")
    all_points = load_map_points(map_file)
    print(f"Loading trajectory from {traj_file}...")
    trajectory = load_trajectory(traj_file)

    if len(all_points) == 0:
        print("No map points, skipping")
        return None
    if len(trajectory) == 0:
        print("No trajectory, skipping")
        return None

    print(f"  Map points: {len(all_points)}, Trajectory poses: {len(trajectory)}")

    # Calculate bounds
    all_xyz = np.array([[t[1], t[2], t[3]] for t in trajectory])
    combined = np.vstack([all_points, all_xyz])
    center = combined.mean(axis=0)
    spread = max(combined.max(axis=0) - combined.min(axis=0))
    if spread < 1e-6:
        spread = 1.0
    scale = min(width, height) * 0.7 / spread

    # Output video
    out_path = os.path.join(slam_dir, "3d_map_animation.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))

    # Progressively reveal points
    # Assign each map point to the nearest trajectory timestamp (simulate building over time)
    traj_times = [t[0] for t in trajectory]
    t_min, t_max = traj_times[0], traj_times[-1]

    # Simple approach: uniformly distribute map points across time
    n_points = len(all_points)
    indices = np.random.permutation(n_points)  # random order for natural look

    step = max(1, len(trajectory) // num_frames)
    frame_poses = trajectory[::step]
    if len(frame_poses) < num_frames:
        frame_poses = trajectory

    actual_frames = len(frame_poses)
    points_per_frame = max(1, n_points // actual_frames)

    print(f"  Rendering {actual_frames} frames...")

    for fi in range(actual_frames):
        # Black background
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        frame[:] = (40, 40, 40)  # dark gray background

        # Points revealed so far
        n_revealed = min((fi + 1) * points_per_frame, n_points)
        revealed_idx = indices[:n_revealed]
        visible_points = all_points[revealed_idx]

        # Current camera position
        _, cx, cy, cz = frame_poses[fi]

        # Project points
        pts_2d = project_3d_to_2d(visible_points, center, width, height, scale, 0, 0)

        # Draw points (color by height/Y value)
        if len(visible_points) > 0:
            y_vals = visible_points[:, 1]
            y_min, y_max = y_vals.min(), y_vals.max()
            if y_max - y_min < 1e-6:
                y_max = y_min + 1

            for pi in range(len(pts_2d)):
                px, py = pts_2d[pi]
                if 0 <= px < width and 0 <= py < height:
                    # Color: blue (low) → green (mid) → red (high)
                    ratio = (y_vals[pi] - y_min) / (y_max - y_min)
                    r = int(255 * ratio)
                    b = int(255 * (1 - ratio))
                    g = int(128 * (1 - abs(ratio - 0.5) * 2))
                    cv2.circle(frame, (px, py), 1, (b, g, r), -1)

        # Draw trajectory path
        for ti in range(1, fi + 1):
            _, x1, y1, z1 = frame_poses[ti - 1]
            _, x2, y2, z2 = frame_poses[ti]
            p1x = int((x1 - center[0]) * scale + width / 2)
            p1y = int((z1 - center[2]) * scale + height / 2)
            p2x = int((x2 - center[0]) * scale + width / 2)
            p2y = int((z2 - center[2]) * scale + height / 2)
            cv2.line(frame, (p1x, p1y), (p2x, p2y), (0, 255, 0), 2)

        # Draw current camera position
        cam_px = int((cx - center[0]) * scale + width / 2)
        cam_py = int((cz - center[2]) * scale + height / 2)
        cv2.rectangle(frame, (cam_px - 6, cam_py - 6), (cam_px + 6, cam_py + 6), (0, 0, 255), 2)

        # Info text
        cv2.putText(frame, f"3D Map Building", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(frame, f"Points: {n_revealed}/{n_points} | KeyFrames: {fi+1}/{actual_frames}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame, f"Cam: ({cx:.2f}, {cy:.2f}, {cz:.2f})",
                    (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        writer.write(frame)

    writer.release()
    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    print(f"  Saved: {out_path} ({size_mb:.1f} MB, {actual_frames} frames)")
    return out_path


def main():
    parser = argparse.ArgumentParser(description="Render 3D map animation from SLAM output")
    parser.add_argument("slam_dir", help="Directory containing MapPoints.txt and trajectory")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=768)
    parser.add_argument("--frames", type=int, default=300, help="Number of animation frames")
    args = parser.parse_args()

    render_3d_animation(args.slam_dir, args.fps, args.width, args.height, args.frames)


if __name__ == "__main__":
    main()
