#!/usr/bin/env python3
"""Prepare a DSO-friendly view of an attacked TUM-format dataset.

DSO (`src/util/DatasetReader.h`) loads frames via `std::sort(files)` over
the `files=` directory and reads timestamps from a sibling `times.txt`
formatted as `<id> <timestamp> <exposure>`. Our raw datasets break both
assumptions:

  * CARLA filenames like `10.800000.png` sort between `108.000002.png`
    and `108.050002.png`, scrambling temporal order.
  * No `times.txt` exists, so every pose DSO writes has timestamp=0
    (see `getImage_internal` line 289) and evaluate_all.py marks the run
    as `no_timestamps`.

Fix: build `<out>/rgb/NNNNNN.png` as symlinks to the real frames in
temporal (rgb.txt) order, and write `<out>/times.txt` with the correct
timestamps in the same order. Pointing DSO at `files=<out>/rgb` then
gives it monotonically-ordered frames with real timestamps.

Usage:
    prepare_dso_input.py <attacked_dataset_dir> <dso_input_out_dir>

Idempotent: existing `<out>` is wiped before rebuild.
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path


def parse_rgb_txt(rgb_txt: Path) -> list[tuple[float, Path]]:
    """Return [(timestamp, absolute_image_path), ...] in file-listed order."""
    entries: list[tuple[float, Path]] = []
    dataset_dir = rgb_txt.parent
    with rgb_txt.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                ts = float(parts[0])
            except ValueError:
                continue
            rel = parts[1]
            img = (dataset_dir / rel).resolve()
            entries.append((ts, img))
    return entries


def main() -> int:
    if len(sys.argv) != 3:
        print(f"usage: {sys.argv[0]} <attacked_dataset_dir> <dso_input_out_dir>",
              file=sys.stderr)
        return 2

    src = Path(sys.argv[1]).resolve()
    out = Path(sys.argv[2]).resolve()

    rgb_txt = src / "rgb.txt"
    if not rgb_txt.exists():
        print(f"error: {rgb_txt} not found", file=sys.stderr)
        return 1

    entries = parse_rgb_txt(rgb_txt)
    if not entries:
        print(f"error: no frames parsed from {rgb_txt}", file=sys.stderr)
        return 1

    # Sort by timestamp so ordering is purely temporal regardless of how
    # rgb.txt happened to be written.
    entries.sort(key=lambda e: e[0])

    # Wipe and rebuild out/rgb/.
    if out.exists():
        shutil.rmtree(out)
    rgb_out = out / "rgb"
    rgb_out.mkdir(parents=True)

    # Zero-padded width large enough for the frame count (min 6 digits).
    width = max(6, len(str(len(entries) - 1)))

    times_lines: list[str] = []
    missing = 0
    for i, (ts, img) in enumerate(entries):
        if not img.exists():
            missing += 1
            continue
        link = rgb_out / f"{i:0{width}d}.png"
        link.symlink_to(img)
        # DSO expects `<id> <timestamp> <exposure>`; exposure unknown -> 0,
        # which is fine in photometric mode=1.
        times_lines.append(f"{i} {ts:.6f} 0")

    (out / "times.txt").write_text("\n".join(times_lines) + "\n")

    print(f"prepare_dso_input: {len(times_lines)} frames linked at {rgb_out}"
          f" ({missing} missing skipped)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
