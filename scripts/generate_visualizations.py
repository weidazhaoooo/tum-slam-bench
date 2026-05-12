#!/usr/bin/env python3
"""
Generate visualization videos for ALL SLAM results in 5_results/.

For each result directory, generates:
  - annotated frames (original image + trajectory overlay + status text)
  - comparison video (original vs attacked side-by-side)

Works with: ORB-SLAM3, DSO, OpenVSLAM, RTAB-Map

Usage:
    python scripts/generate_visualizations.py                    # process all
    python scripts/generate_visualizations.py <result_dir>       # process one
"""

import os
import sys
import glob
import csv
import numpy as np
import cv2
from pathlib import Path

BASE = os.environ.get("VSLAM_BASE", "/home/weida/v_slam_dataset")
RESULTS = os.environ.get("SLAM_RESULTS_DIR", os.path.join(BASE, "5_results"))
ORIGIN = os.environ.get("TUM_DATA_DIR", os.path.join(BASE, "3_origin_running_video_tum_format_data"))
# Optional: only used if you have perturbed/overlayed copies of TUM datasets.
# Safe to leave at default — accessed lazily via os.path.exists().
OVERLAYED = os.environ.get("OVERLAYED_DATA_DIR", os.path.join(BASE, "4_overlayed_tum_format_data"))


def load_trajectory_tum(filepath):
    """Load TUM-format trajectory: timestamp tx ty tz qx qy qz qw"""
    traj = {}
    if not os.path.exists(filepath):
        return traj
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                ts = float(parts[0])
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                traj[ts] = (tx, ty, tz)
    return traj


def find_rgb_dir(run_name):
    """Find the RGB directory for a given run name."""
    # Try overlayed data first
    overlayed = os.path.join(OVERLAYED, run_name, "rgb")
    if os.path.isdir(overlayed):
        return overlayed

    # Try to extract original dataset name
    # e.g. rgbd_dataset_freiburg1_xyz_ov1_i0.5 → rgbd_dataset_freiburg1_xyz
    for orig in sorted(os.listdir(ORIGIN)):
        if run_name.startswith(orig):
            rgb = os.path.join(ORIGIN, orig, "rgb")
            if os.path.isdir(rgb):
                return rgb
    return None


def draw_trajectory_on_frame(frame, traj_points, current_idx, slam_name, status="OK"):
    """Draw trajectory path and info on frame."""
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # Draw info box
    cv2.rectangle(overlay, (5, 5), (300, 70), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
    cv2.putText(frame, f"{slam_name}", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"Frame: {current_idx} | Status: {status}", (10, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Draw mini trajectory plot in bottom-left corner
    if len(traj_points) >= 2:
        pts = np.array(traj_points[:current_idx + 1])
        if len(pts) >= 2:
            # Use X-Z plane (top-down view)
            xs = pts[:, 0]
            zs = pts[:, 2] if pts.shape[1] > 2 else pts[:, 1]

            # Normalize to fit in a small box
            plot_size = min(h, w) // 4
            margin = 10
            px = margin
            py = h - plot_size - margin

            # Draw background
            cv2.rectangle(frame, (px, py), (px + plot_size, py + plot_size), (0, 0, 0), -1)
            cv2.rectangle(frame, (px, py), (px + plot_size, py + plot_size), (100, 100, 100), 1)

            if xs.max() - xs.min() > 1e-6 and zs.max() - zs.min() > 1e-6:
                x_norm = ((xs - xs.min()) / (xs.max() - xs.min()) * (plot_size - 20) + 10).astype(int)
                z_norm = ((zs - zs.min()) / (zs.max() - zs.min()) * (plot_size - 20) + 10).astype(int)

                for i in range(1, len(x_norm)):
                    cv2.line(frame,
                             (px + x_norm[i-1], py + z_norm[i-1]),
                             (px + x_norm[i], py + z_norm[i]),
                             (0, 255, 0), 1)

                # Current position
                cv2.circle(frame, (px + x_norm[-1], py + z_norm[-1]), 4, (0, 0, 255), -1)

    return frame


def generate_slam_video(result_dir, slam_subdir, run_name, fps=30):
    """Generate visualization video for a specific SLAM result."""
    slam_path = os.path.join(result_dir, slam_subdir)
    if not os.path.isdir(slam_path):
        return

    slam_name = slam_subdir.upper()

    # Already has comparison video? (ORB-SLAM3 with viewer)
    existing_comp = os.path.join(slam_path, "comparison.mp4")
    if os.path.exists(existing_comp):
        print(f"  [{slam_name}] Already has comparison.mp4, skipping")
        return

    # Find trajectory
    traj_file = None
    for name in ["CameraTrajectory.txt", "frame_trajectory.txt", "rtabmap_poses.txt"]:
        f = os.path.join(slam_path, name)
        if os.path.exists(f):
            traj_file = f
            break

    if not traj_file:
        print(f"  [{slam_name}] No trajectory file found, skipping")
        return

    traj = load_trajectory_tum(traj_file)
    if not traj:
        print(f"  [{slam_name}] Empty trajectory, skipping")
        return

    # Find RGB images
    rgb_dir = find_rgb_dir(run_name)
    if not rgb_dir:
        print(f"  [{slam_name}] No RGB directory found for {run_name}, skipping")
        return

    rgb_files = sorted(glob.glob(os.path.join(rgb_dir, "*.png")))
    if not rgb_files:
        print(f"  [{slam_name}] No PNG files in {rgb_dir}, skipping")
        return

    # Build trajectory points list (ordered by timestamp)
    traj_timestamps = sorted(traj.keys())
    traj_points = [traj[t] for t in traj_timestamps]

    # Output video
    vis_dir = os.path.join(slam_path, "visualization")
    os.makedirs(vis_dir, exist_ok=True)
    out_video = os.path.join(slam_path, "slam_visualization.mp4")

    # Read first frame to get dimensions
    sample = cv2.imread(rgb_files[0])
    if sample is None:
        return
    h, w = sample.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_video, fourcc, fps, (w, h))

    # Match frames to trajectory
    traj_idx = 0
    for i, rgb_file in enumerate(rgb_files):
        frame = cv2.imread(rgb_file)
        if frame is None:
            continue

        # Get frame timestamp from filename
        ts_str = os.path.splitext(os.path.basename(rgb_file))[0]
        try:
            frame_ts = float(ts_str)
        except ValueError:
            frame_ts = i / fps

        # Find closest trajectory point
        while traj_idx < len(traj_timestamps) - 1 and traj_timestamps[traj_idx] < frame_ts:
            traj_idx += 1

        status = "Tracking" if traj_idx < len(traj_timestamps) else "Lost"
        frame = draw_trajectory_on_frame(frame, traj_points, traj_idx, slam_name, status)

        writer.write(frame)

        # Save some sample frames
        if i % (len(rgb_files) // 10 + 1) == 0:
            cv2.imwrite(os.path.join(vis_dir, f"frame_{i:06d}.png"), frame)

    writer.release()
    size_mb = os.path.getsize(out_video) / (1024 * 1024)
    print(f"  [{slam_name}] Generated {out_video} ({size_mb:.1f} MB, {len(rgb_files)} frames)")


def process_result_dir(result_dir):
    """Process all SLAM subdirectories in a result directory."""
    run_name = os.path.basename(result_dir)
    print(f"\n=== {run_name} ===")

    for slam_sub in ["orbslam3", "dso", "openvslam", "rtabmap"]:
        slam_path = os.path.join(result_dir, slam_sub)
        if os.path.isdir(slam_path):
            generate_slam_video(result_dir, slam_sub, run_name)

    # Also check if trajectory files are directly in result_dir (old format)
    for traj_name in ["CameraTrajectory.txt"]:
        if os.path.exists(os.path.join(result_dir, traj_name)) and \
           not os.path.isdir(os.path.join(result_dir, "orbslam3")):
            print(f"  [Legacy] Found {traj_name} in root, skipping (use SLAM subdirs)")


def main():
    if len(sys.argv) > 1:
        # Process specific directory
        process_result_dir(sys.argv[1])
    else:
        # Process all result directories
        for d in sorted(os.listdir(RESULTS)):
            full = os.path.join(RESULTS, d)
            if os.path.isdir(full) and ("_ov" in d or "original" in d):
                process_result_dir(full)


if __name__ == "__main__":
    main()
