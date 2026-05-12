#!/usr/bin/env python3
"""
Prepare synchronized rgb_sync/ and depth_sync/ folders for RTAB-Map
from a TUM RGB-D format dataset.

Usage:
    python prepare_rtabmap_sync.py <dataset_path>
"""
import os
import sys
import shutil

def read_timestamps(filepath):
    """Read TUM timestamp file, return dict {timestamp: filename}"""
    data = {}
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 2:
                data[float(parts[0])] = parts[1]
    return data

def associate(rgb_dict, depth_dict, max_diff=0.02):
    """Associate RGB and depth by closest timestamp."""
    rgb_stamps = sorted(rgb_dict.keys())
    depth_stamps = sorted(depth_dict.keys())
    matches = []
    j = 0
    for rs in rgb_stamps:
        while j < len(depth_stamps) - 1 and abs(depth_stamps[j+1] - rs) < abs(depth_stamps[j] - rs):
            j += 1
        if j < len(depth_stamps) and abs(depth_stamps[j] - rs) < max_diff:
            matches.append((rs, depth_stamps[j]))
    return matches

def prepare_sync(dataset_path):
    rgb_txt = os.path.join(dataset_path, 'rgb.txt')
    depth_txt = os.path.join(dataset_path, 'depth.txt')

    if not os.path.exists(rgb_txt) or not os.path.exists(depth_txt):
        print(f"Error: {dataset_path} missing rgb.txt or depth.txt")
        return False

    rgb_sync = os.path.join(dataset_path, 'rgb_sync')
    depth_sync = os.path.join(dataset_path, 'depth_sync')

    if os.path.exists(rgb_sync) and len(os.listdir(rgb_sync)) > 0:
        print(f"Already synced: {len(os.listdir(rgb_sync))} frames in rgb_sync/")
        return True

    os.makedirs(rgb_sync, exist_ok=True)
    os.makedirs(depth_sync, exist_ok=True)

    rgb_dict = read_timestamps(rgb_txt)
    depth_dict = read_timestamps(depth_txt)
    matches = associate(rgb_dict, depth_dict)

    print(f"RGB: {len(rgb_dict)}, Depth: {len(depth_dict)}, Matches: {len(matches)}")

    for rgb_ts, depth_ts in matches:
        rgb_file = rgb_dict[rgb_ts]  # e.g. rgb/1305031102.175304.png
        depth_file = depth_dict[depth_ts]

        rgb_src = os.path.join(dataset_path, rgb_file)
        depth_src = os.path.join(dataset_path, depth_file)

        rgb_name = os.path.basename(rgb_file)
        depth_name = os.path.basename(depth_file)

        if os.path.exists(rgb_src) and os.path.exists(depth_src):
            # Symlink instead of copy to save space
            os.symlink(os.path.abspath(rgb_src), os.path.join(rgb_sync, rgb_name))
            os.symlink(os.path.abspath(depth_src), os.path.join(depth_sync, rgb_name))

    print(f"Created {len(matches)} synced frame pairs")
    return True

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python prepare_rtabmap_sync.py <dataset_path>")
        sys.exit(1)
    prepare_sync(sys.argv[1])
