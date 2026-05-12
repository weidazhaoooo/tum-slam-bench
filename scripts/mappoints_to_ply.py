#!/usr/bin/env python3
"""
Convert ORB-SLAM3 MapPoints.txt -> standard ASCII .ply.

MapPoints.txt format (from System::SaveMapPoints):
    # x y z first_kf_id first_frame_id n_obs n_visible n_found
    <floats...>

Only the xyz columns are kept in the output .ply.
"""

import argparse
from pathlib import Path


def convert(src: Path, dst: Path) -> int:
    pts = []
    with src.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cols = line.split()
            if len(cols) < 3:
                continue
            try:
                pts.append((float(cols[0]), float(cols[1]), float(cols[2])))
            except ValueError:
                continue

    with dst.open("w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for x, y, z in pts:
            f.write(f"{x:.7f} {y:.7f} {z:.7f}\n")
    return len(pts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path, help="MapPoints.txt path")
    ap.add_argument("dst", type=Path, nargs="?",
                    help="output .ply (default: same dir, MapPoints.ply)")
    args = ap.parse_args()

    if not args.src.exists():
        raise SystemExit(f"missing: {args.src}")
    dst = args.dst if args.dst else args.src.with_suffix(".ply")
    n = convert(args.src, dst)
    print(f"[ply] {n} points -> {dst}")


if __name__ == "__main__":
    main()
