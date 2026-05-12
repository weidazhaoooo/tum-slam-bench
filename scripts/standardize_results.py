#!/usr/bin/env python3
"""
Standardize SLAM run outputs and append a summary line to results_index.csv

Usage:
  python3 standardize_results.py --save-dir /path/to/results/XYZ --run-name RUN_ID --results-root /path/to/results_root

This script is intentionally conservative: it only inspects known subfolders
and copies/records existing trajectory files into a manifest. It then appends
one CSV line to `results_index.csv` at the `results_root` location.
"""
import argparse
import csv
import json
import os
import subprocess
import sys
from datetime import datetime


def short_git_rev(path):
    try:
        out = subprocess.check_output(['git', '-C', path, 'rev-parse', '--short', 'HEAD'], stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        return 'unknown'


def collect_system(save_dir, system):
    d = os.path.join(save_dir, system)
    info = {'present': False, 'traj': None, 'lines': 0}
    if not os.path.isdir(d):
        return info
    # Common trajectory filenames
    candidates = [
        'CameraTrajectory.txt',
        'frame_trajectory.txt',
        'result.txt',
        'orbslam3_output.txt',
        'orbslam2_output.txt'
    ]
    for c in candidates:
        p = os.path.join(d, c)
        if os.path.isfile(p):
            info['present'] = True
            info['traj'] = os.path.relpath(p, save_dir)
            try:
                info['lines'] = sum(1 for _ in open(p, 'r'))
            except Exception:
                info['lines'] = 0
            return info
    # Also check top-level
    for c in ['CameraTrajectory.txt', 'KeyFrameTrajectory.txt', 'MapPoints.txt']:
        p = os.path.join(save_dir, c)
        if os.path.isfile(p):
            info['present'] = True
            info['traj'] = os.path.relpath(p, save_dir)
            try:
                info['lines'] = sum(1 for _ in open(p, 'r'))
            except Exception:
                info['lines'] = 0
            return info
    return info


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save-dir', required=True)
    parser.add_argument('--run-name', required=True)
    parser.add_argument('--results-root', required=True)
    args = parser.parse_args()

    save = args.save_dir
    results_root = args.results_root
    run_name = args.run_name

    systems = ['orbslam3', 'orbslam2', 'dso', 'openvslam', 'rtabmap', 'midas', 'dav2']
    found = {}
    for s in systems:
        found[s] = collect_system(save, s)

    manifest = {
        'run_name': run_name,
        'save_dir': os.path.abspath(save),
        'timestamp': datetime.utcnow().isoformat() + 'Z',
        'git_rev': short_git_rev(os.path.dirname(os.path.dirname(__file__))),
        'systems': found
    }

    # Write manifest.json next to save dir
    try:
        mf = os.path.join(save, 'manifest.json')
        with open(mf, 'w') as f:
            json.dump(manifest, f, indent=2)
        print(f"Wrote manifest: {mf}")
    except Exception as e:
        print(f"Failed to write manifest: {e}", file=sys.stderr)

    # Append a CSV summary line to results_index.csv at results_root
    idx = os.path.join(results_root, 'results_index.csv')
    header = ['run_name', 'save_dir', 'timestamp', 'git_rev', 'systems_present']
    line = [run_name, os.path.abspath(save), manifest['timestamp'], manifest['git_rev'], 
            ';'.join([s for s in systems if found[s]['present']])]

    try:
        new_file = not os.path.isfile(idx)
        with open(idx, 'a', newline='') as csvf:
            w = csv.writer(csvf)
            if new_file:
                w.writerow(header)
            w.writerow(line)
        print(f"Appended index: {idx}")
    except Exception as e:
        print(f"Failed to append index: {e}", file=sys.stderr)


if __name__ == '__main__':
    main()
