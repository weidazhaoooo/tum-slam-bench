#!/usr/bin/env python3
"""
Post-hoc Pangolin-style viewer for ORB-SLAM3 output.

Reads:
    <slam_dir>/MapPoints.txt          (or MapPoints.ply)
    <slam_dir>/KeyFrameTrajectory.txt (or CameraTrajectory.txt)

Shows interactively (drag to rotate, wheel to zoom):
    - black map points
    - blue camera frustums at each keyframe pose
    - green trajectory line
    - thicker green frustum at the final pose

Requires: pip install open3d
"""

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def load_points(slam_dir: Path) -> np.ndarray:
    ply = slam_dir / "MapPoints.ply"
    if ply.exists():
        return np.asarray(o3d.io.read_point_cloud(str(ply)).points)
    txt = slam_dir / "MapPoints.txt"
    if txt.exists():
        rows = []
        for line in txt.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split()
            if len(cols) >= 3:
                rows.append([float(cols[0]), float(cols[1]), float(cols[2])])
        return np.array(rows) if rows else np.zeros((0, 3))
    raise SystemExit(f"no MapPoints.{{ply,txt}} in {slam_dir}")


def load_trajectory(slam_dir: Path) -> np.ndarray:
    """Return Nx8 array: [t, tx, ty, tz, qx, qy, qz, qw]."""
    for name in ("KeyFrameTrajectory.txt", "CameraTrajectory.txt"):
        p = slam_dir / name
        if not p.exists():
            continue
        rows = []
        for line in p.read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split()
            if len(cols) >= 8:
                rows.append([float(c) for c in cols[:8]])
        if rows:
            return np.array(rows)
    raise SystemExit(f"no trajectory in {slam_dir}")


def quat_to_R(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def make_frustum(t, R, scale, color):
    """Camera frustum (pyramid) as open3d LineSet at pose (R|t)."""
    w, h, d = 0.6 * scale, 0.45 * scale, scale
    corners_cam = np.array([
        [0, 0, 0],
        [w, h, d], [w, -h, d], [-w, -h, d], [-w, h, d],
    ])
    pts = (R @ corners_cam.T).T + t
    lines = [[0, 1], [0, 2], [0, 3], [0, 4],
             [1, 2], [2, 3], [3, 4], [4, 1]]
    ls = o3d.geometry.LineSet()
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector([color] * len(lines))
    return ls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("slam_dir", type=Path)
    ap.add_argument("--frustum-scale", type=float, default=None,
                    help="frustum size; defaults to 2%% of scene span")
    ap.add_argument("--every", type=int, default=1,
                    help="draw every Nth keyframe frustum (default: all)")
    args = ap.parse_args()

    pts = load_points(args.slam_dir)
    traj = load_trajectory(args.slam_dir)
    print(f"[load] {len(pts)} points, {len(traj)} poses from {args.slam_dir}")

    cam_centers = traj[:, 1:4]
    span = float(np.linalg.norm(cam_centers.max(0) - cam_centers.min(0)))
    if span < 1e-6:
        span = 1.0
    fs = args.frustum_scale if args.frustum_scale else max(0.02 * span, 1e-3)

    geoms = []

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    pcd.paint_uniform_color([0.05, 0.05, 0.05])
    geoms.append(pcd)

    line_pts = cam_centers
    line_idx = [[i, i + 1] for i in range(len(line_pts) - 1)]
    if line_idx:
        traj_ls = o3d.geometry.LineSet()
        traj_ls.points = o3d.utility.Vector3dVector(line_pts)
        traj_ls.lines = o3d.utility.Vector2iVector(line_idx)
        traj_ls.colors = o3d.utility.Vector3dVector([[0.0, 0.7, 0.0]] * len(line_idx))
        geoms.append(traj_ls)

    blue = [0.0, 0.3, 1.0]
    for i, row in enumerate(traj[::args.every]):
        t = row[1:4]
        R = quat_to_R(*row[4:8])
        geoms.append(make_frustum(t, R, fs * 0.7, blue))

    last = traj[-1]
    geoms.append(make_frustum(last[1:4], quat_to_R(*last[4:8]), fs * 1.4, [0.0, 0.9, 0.0]))

    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=fs * 2))

    print("[viewer] drag=rotate, wheel=zoom, right-drag=pan, H=help, Q=quit")
    o3d.visualization.draw_geometries(
        geoms,
        window_name=f"ORB-SLAM3 map: {args.slam_dir.name}",
        width=1280, height=800,
    )


if __name__ == "__main__":
    main()
