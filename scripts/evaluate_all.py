#!/usr/bin/env python3
"""
Quantitative evaluation of V-SLAM overlay attack results.

Computes ATE, RPE, and tracking rate for all results against groundtruth.
Outputs:
  - 5_results/evaluation_summary.csv   (per-run metrics)
  - 5_results/evaluation_plots/        (comparison figures)
"""

import os
import sys
import csv
import copy
import re
import warnings
from pathlib import Path
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.core import lie_algebra
from evo.core.units import Unit

# ─── paths (env-var overridable; defaults match dev workspace) ───
import os
BASE = Path(os.environ.get("VSLAM_BASE", "/home/weida/v_slam_dataset"))
RESULTS = Path(os.environ.get("SLAM_RESULTS_DIR", BASE / "5_results"))
ORIGIN = Path(os.environ.get("TUM_DATA_DIR", BASE / "3_origin_running_video_tum_format_data"))
PLOT_DIR = RESULTS / "evaluation_plots"
CSV_PATH = RESULTS / "evaluation_summary.csv"

PLOT_DIR.mkdir(exist_ok=True)


# ─── helpers ───

def count_camera_frames(dataset_name):
    """Count actual camera frames (rgb images) for a dataset."""
    base = re.sub(r"_(ov\d+_i[\d.]+|original|overlay_\d+|laser_\w+|freeze_\w+)$", "", dataset_name)
    rgb_dir = ORIGIN / base / "rgb"
    if rgb_dir.exists():
        return len(list(rgb_dir.glob("*.png"))) or len(list(rgb_dir.glob("*.jpg")))
    rgb_txt = ORIGIN / base / "rgb.txt"
    if rgb_txt.exists():
        count = 0
        with open(rgb_txt) as f:
            for line in f:
                if not line.strip().startswith("#") and line.strip():
                    count += 1
        return count
    return 0


def has_valid_timestamps(filepath):
    """Check if trajectory file has non-zero diverse timestamps."""
    stamps = set()
    with open(filepath) as f:
        for i, line in enumerate(f):
            if i > 10:
                break
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 8:
                try:
                    stamps.add(float(parts[0]))
                except ValueError:
                    pass
    return len(stamps) > 1


def read_tum_trajectory(filepath):
    """Read TUM-format trajectory file, skip comments and empty lines."""
    stamps, xyz, quat = [], [], []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                t = float(parts[0])
                tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            except ValueError:
                continue
            stamps.append(t)
            xyz.append([tx, ty, tz])
            quat.append([qx, qy, qz, qw])

    if len(stamps) < 2:
        return None

    return PoseTrajectory3D(
        positions_xyz=np.array(xyz),
        orientations_quat_wxyz=np.array(quat)[:, [3, 0, 1, 2]],  # xyzw -> wxyz
        timestamps=np.array(stamps),
    )


def find_groundtruth(dataset_name):
    """Locate groundtruth.txt for a dataset."""
    # strip overlay/intensity suffix to get base dataset name
    # e.g. "rgbd_dataset_freiburg1_xyz_ov1_i0.5" -> "rgbd_dataset_freiburg1_xyz"
    # e.g. "carla_town01_clear_01_ov2_i0.7" -> "carla_town01_clear_01"
    base = re.sub(r"_(ov\d+_i[\d.]+|original|overlay_\d+|laser_\w+|freeze_\w+)$", "", dataset_name)
    gt_path = ORIGIN / base / "groundtruth.txt"
    if gt_path.exists():
        return gt_path
    return None


def is_monocular(dataset_name, slam_name=None):
    """Check if a given result is monocular.

    CARLA datasets are always monocular; DSO is a monocular system even
    on TUM RGB-D datasets, so its trajectory has arbitrary scale and must
    be Sim3-aligned. Everything else (ORB-SLAM2/3, RTAB-Map, OpenVSLAM
    in RGB-D mode) is SE3-aligned on TUM.
    """
    if slam_name == "dso":
        return True
    return "carla" in dataset_name


def compute_ate(traj_ref, traj_est, monocular=False):
    """Compute ATE with SE3 (rgbd) or Sim3 (mono) alignment."""
    traj_ref_sync, traj_est_sync = sync.associate_trajectories(
        traj_ref, traj_est, max_diff=0.05
    )
    if traj_est_sync.num_poses < 3:
        return None, None, 0

    traj_est_aligned = copy.deepcopy(traj_est_sync)
    if monocular:
        r, t, s = traj_est_aligned.align(traj_ref_sync, correct_scale=True)
    else:
        r, t, s = traj_est_aligned.align(traj_ref_sync, correct_scale=False)

    data = (traj_ref_sync, traj_est_aligned)
    ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    ape_metric.process_data(data)
    stats = ape_metric.get_all_statistics()

    return stats, s, traj_est_sync.num_poses


def compute_rpe(traj_ref, traj_est, monocular=False, delta=1.0):
    """Compute RPE (translational + rotational)."""
    traj_ref_sync, traj_est_sync = sync.associate_trajectories(
        traj_ref, traj_est, max_diff=0.05
    )
    if traj_est_sync.num_poses < 3:
        return None, None

    traj_est_aligned = copy.deepcopy(traj_est_sync)
    if monocular:
        traj_est_aligned.align(traj_ref_sync, correct_scale=True)
    else:
        traj_est_aligned.align(traj_ref_sync, correct_scale=False)

    data = (traj_ref_sync, traj_est_aligned)

    # translational RPE
    rpe_trans = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=delta, delta_unit=Unit.frames, all_pairs=False,
    )
    rpe_trans.process_data(data)
    trans_stats = rpe_trans.get_all_statistics()

    # rotational RPE
    rpe_rot = metrics.RPE(
        metrics.PoseRelation.rotation_angle_deg,
        delta=delta, delta_unit=Unit.frames, all_pairs=False,
    )
    rpe_rot.process_data(data)
    rot_stats = rpe_rot.get_all_statistics()

    return trans_stats, rot_stats


def parse_result_name(name):
    """Parse result directory name into components."""
    m = re.match(r"^(.+?)_(ov(\d+)_i([\d.]+)|original)$", name)
    if not m:
        return None
    dataset_base = m.group(1)
    if m.group(2) == "original":
        overlay, intensity = "original", 0.0
    else:
        overlay = int(m.group(3))
        intensity = float(m.group(4))
    return {
        "dataset": dataset_base,
        "overlay": overlay,
        "intensity": intensity,
        "is_original": m.group(2) == "original",
    }


# ─── main evaluation loop ───

def run_evaluation():
    rows = []
    result_dirs = sorted([d for d in RESULTS.iterdir() if d.is_dir()])

    total = len(result_dirs)
    processed = 0

    for rdir in result_dirs:
        name = rdir.name
        if name in ("evaluation_plots",):
            continue

        parsed = parse_result_name(name)
        if parsed is None:
            continue

        gt_path = find_groundtruth(name)
        if gt_path is None:
            continue

        traj_gt = read_tum_trajectory(str(gt_path))
        if traj_gt is None:
            continue

        gt_frames = traj_gt.num_poses
        cam_frames = count_camera_frames(name) or gt_frames

        # find trajectory files: check SLAM sub-dirs first, then root
        traj_files = {}

        for slam_dir in sorted(rdir.iterdir()):
            if not slam_dir.is_dir():
                continue
            sname = slam_dir.name
            if sname in ("annotated", "map", "midas", "depth_anything_v2"):
                continue
            cam_traj = slam_dir / "CameraTrajectory.txt"
            if cam_traj.exists() and cam_traj.stat().st_size > 0:
                traj_files[sname] = cam_traj

        # root-level trajectory (legacy format, treat as orbslam3)
        root_cam = rdir / "CameraTrajectory.txt"
        if root_cam.exists() and root_cam.stat().st_size > 0 and not traj_files:
            traj_files["orbslam3"] = root_cam

        for slam_name, traj_path in traj_files.items():
            # Per-result mono check: CARLA is always mono; DSO is mono
            # even when the dataset is TUM RGB-D.
            mono = is_monocular(name, slam_name)

            # Skip trajectories without valid timestamps (e.g. DSO outputs all-zero)
            if not has_valid_timestamps(str(traj_path)):
                traj_est = read_tum_trajectory(str(traj_path))
                est_frames = traj_est.num_poses if traj_est else 0
                rows.append({
                    "dataset": parsed["dataset"],
                    "overlay": parsed["overlay"],
                    "intensity": parsed["intensity"],
                    "slam": slam_name,
                    "ate_rmse": np.nan,
                    "ate_mean": np.nan,
                    "ate_median": np.nan,
                    "ate_std": np.nan,
                    "rpe_trans_rmse": np.nan,
                    "rpe_rot_rmse": np.nan,
                    "tracked_frames": est_frames,
                    "cam_frames": cam_frames,
                    "tracking_rate": est_frames / cam_frames if cam_frames > 0 else 0.0,
                    "scale": np.nan,
                    "status": "no_timestamps",
                })
                continue

            traj_est = read_tum_trajectory(str(traj_path))
            if traj_est is None or traj_est.num_poses < 3:
                rows.append({
                    "dataset": parsed["dataset"],
                    "overlay": parsed["overlay"],
                    "intensity": parsed["intensity"],
                    "slam": slam_name,
                    "ate_rmse": np.nan,
                    "ate_mean": np.nan,
                    "ate_median": np.nan,
                    "ate_std": np.nan,
                    "rpe_trans_rmse": np.nan,
                    "rpe_rot_rmse": np.nan,
                    "tracked_frames": traj_est.num_poses if traj_est else 0,
                    "cam_frames": cam_frames,
                    "tracking_rate": 0.0,
                    "scale": np.nan,
                    "status": "too_few_poses",
                })
                continue

            try:
                ate_stats, scale, matched = compute_ate(traj_gt, traj_est, mono)
            except Exception as e:
                ate_stats, scale, matched = None, np.nan, 0

            try:
                rpe_t_stats, rpe_r_stats = compute_rpe(traj_gt, traj_est, mono)
            except Exception:
                rpe_t_stats, rpe_r_stats = None, None

            tracking_rate = matched / cam_frames if cam_frames > 0 else 0.0

            row = {
                "dataset": parsed["dataset"],
                "overlay": parsed["overlay"],
                "intensity": parsed["intensity"],
                "slam": slam_name,
                "ate_rmse": ate_stats["rmse"] if ate_stats else np.nan,
                "ate_mean": ate_stats["mean"] if ate_stats else np.nan,
                "ate_median": ate_stats["median"] if ate_stats else np.nan,
                "ate_std": ate_stats["std"] if ate_stats else np.nan,
                "rpe_trans_rmse": rpe_t_stats["rmse"] if rpe_t_stats else np.nan,
                "rpe_rot_rmse": rpe_r_stats["rmse"] if rpe_r_stats else np.nan,
                "tracked_frames": traj_est.num_poses,
                "cam_frames": cam_frames,
                "tracking_rate": tracking_rate,
                "scale": scale if scale else np.nan,
                "status": "ok" if ate_stats else "eval_failed",
            }
            rows.append(row)

        processed += 1
        if processed % 20 == 0:
            print(f"  [{processed}/{total}] processed {name}")

    return rows


def generate_plots(df):
    """Generate comparison plots."""
    plt.rcParams.update({"font.size": 10, "figure.dpi": 150})

    # ── 1. ATE RMSE heatmap per dataset×SLAM: overlay vs intensity ──
    for dataset in df["dataset"].unique():
        ds = df[(df["dataset"] == dataset) & (df["overlay"] != "original")]
        if ds.empty:
            continue

        for slam in ds["slam"].unique():
            sub = ds[ds["slam"] == slam]
            if sub.empty:
                continue

            pivot = sub.pivot_table(
                values="ate_rmse", index="overlay", columns="intensity", aggfunc="mean"
            )
            if pivot.empty:
                continue

            # get baseline
            base = df[(df["dataset"] == dataset) & (df["slam"] == slam) & (df["overlay"] == "original")]
            baseline_ate = base["ate_rmse"].mean() if not base.empty else np.nan

            fig, ax = plt.subplots(figsize=(6, 4))
            im = ax.imshow(pivot.values, cmap="YlOrRd", aspect="auto")
            ax.set_xticks(range(len(pivot.columns)))
            ax.set_xticklabels([f"{c}" for c in pivot.columns])
            ax.set_yticks(range(len(pivot.index)))
            ax.set_yticklabels([f"ov{i}" for i in pivot.index])
            ax.set_xlabel("Intensity")
            ax.set_ylabel("Overlay")

            for i in range(len(pivot.index)):
                for j in range(len(pivot.columns)):
                    v = pivot.values[i, j]
                    if not np.isnan(v):
                        ax.text(j, i, f"{v:.4f}", ha="center", va="center", fontsize=8)

            title = f"ATE RMSE (m) - {dataset} / {slam}"
            if not np.isnan(baseline_ate):
                title += f"\nBaseline: {baseline_ate:.4f}"
            ax.set_title(title)
            fig.colorbar(im, ax=ax)
            fig.tight_layout()
            fig.savefig(PLOT_DIR / f"ate_heatmap_{dataset}_{slam}.png")
            plt.close(fig)

    # ── 2. Tracking rate heatmap ──
    for dataset in df["dataset"].unique():
        ds = df[(df["dataset"] == dataset) & (df["overlay"] != "original")]
        if ds.empty:
            continue
        for slam in ds["slam"].unique():
            sub = ds[ds["slam"] == slam]
            pivot = sub.pivot_table(
                values="tracking_rate", index="overlay", columns="intensity", aggfunc="mean"
            )
            if pivot.empty:
                continue

            fig, ax = plt.subplots(figsize=(6, 4))
            im = ax.imshow(pivot.values, cmap="RdYlGn", aspect="auto", vmin=0, vmax=1)
            ax.set_xticks(range(len(pivot.columns)))
            ax.set_xticklabels([f"{c}" for c in pivot.columns])
            ax.set_yticks(range(len(pivot.index)))
            ax.set_yticklabels([f"ov{i}" for i in pivot.index])
            ax.set_xlabel("Intensity")
            ax.set_ylabel("Overlay")

            for i in range(len(pivot.index)):
                for j in range(len(pivot.columns)):
                    v = pivot.values[i, j]
                    if not np.isnan(v):
                        ax.text(j, i, f"{v:.1%}", ha="center", va="center", fontsize=8)

            ax.set_title(f"Tracking Rate - {dataset} / {slam}")
            fig.colorbar(im, ax=ax)
            fig.tight_layout()
            fig.savefig(PLOT_DIR / f"tracking_heatmap_{dataset}_{slam}.png")
            plt.close(fig)

    # ── 3. Bar chart: ATE per SLAM system (all overlays aggregated) ──
    for dataset in df["dataset"].unique():
        ds = df[df["dataset"] == dataset]
        if ds.empty:
            continue

        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        # ATE by SLAM
        slam_ate = ds.groupby(["slam", "overlay"])["ate_rmse"].mean().reset_index()
        slams = sorted(ds["slam"].unique())

        for ax, metric, label in [
            (axes[0], "ate_rmse", "ATE RMSE (m)"),
            (axes[1], "tracking_rate", "Tracking Rate"),
        ]:
            orig = ds[ds["overlay"] == "original"].groupby("slam")[metric].mean()
            attack = ds[ds["overlay"] != "original"].groupby("slam")[metric].mean()

            x = np.arange(len(slams))
            w = 0.35
            bars1 = [orig.get(s, np.nan) for s in slams]
            bars2 = [attack.get(s, np.nan) for s in slams]

            ax.bar(x - w/2, bars1, w, label="Original", color="#4CAF50", alpha=0.8)
            ax.bar(x + w/2, bars2, w, label="Attack (avg)", color="#F44336", alpha=0.8)
            ax.set_xticks(x)
            ax.set_xticklabels(slams, rotation=30, ha="right")
            ax.set_ylabel(label)
            ax.legend()
            ax.set_title(f"{dataset}")

        fig.tight_layout()
        fig.savefig(PLOT_DIR / f"slam_comparison_{dataset}.png")
        plt.close(fig)

    # ── 4. Intensity effect line plot ──
    for dataset in df["dataset"].unique():
        ds = df[(df["dataset"] == dataset) & (df["overlay"] != "original")]
        if ds.empty:
            continue

        slams = sorted(ds["slam"].unique())
        fig, ax = plt.subplots(figsize=(8, 5))

        for slam in slams:
            sub = ds[ds["slam"] == slam]
            means = sub.groupby("intensity")["ate_rmse"].mean().sort_index()
            if not means.empty:
                ax.plot(means.index, means.values, marker="o", label=slam)

        # baseline
        for slam in slams:
            base = df[(df["dataset"] == dataset) & (df["slam"] == slam) & (df["overlay"] == "original")]
            if not base.empty:
                bv = base["ate_rmse"].mean()
                ax.axhline(y=bv, linestyle="--", alpha=0.5, label=f"{slam} baseline")

        ax.set_xlabel("Overlay Intensity")
        ax.set_ylabel("ATE RMSE (m)")
        ax.set_title(f"Attack Intensity vs ATE - {dataset}")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(PLOT_DIR / f"intensity_effect_{dataset}.png")
        plt.close(fig)

    # ── 5. Per-overlay comparison across SLAM systems ──
    tum_datasets = [d for d in df["dataset"].unique() if "freiburg" in d]
    for dataset in tum_datasets:
        ds = df[(df["dataset"] == dataset) & (df["overlay"] != "original")]
        if ds.empty:
            continue
        slams = sorted(ds["slam"].unique())
        overlays = sorted(ds["overlay"].unique())

        fig, ax = plt.subplots(figsize=(10, 5))
        x = np.arange(len(overlays))
        w = 0.8 / max(len(slams), 1)

        for i, slam in enumerate(slams):
            vals = []
            for ov in overlays:
                sub = ds[(ds["slam"] == slam) & (ds["overlay"] == ov)]
                vals.append(sub["ate_rmse"].mean() if not sub.empty else np.nan)
            ax.bar(x + i * w - 0.4 + w/2, vals, w, label=slam, alpha=0.85)

        ax.set_xticks(x)
        ax.set_xticklabels([f"ov{o}" for o in overlays])
        ax.set_xlabel("Overlay Pattern")
        ax.set_ylabel("ATE RMSE (m)")
        ax.set_title(f"Per-Overlay ATE Comparison - {dataset}")
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(PLOT_DIR / f"overlay_comparison_{dataset}.png")
        plt.close(fig)


def print_summary(df, df_all):
    """Print summary tables to stdout."""
    print("\n" + "=" * 90)
    print("EVALUATION SUMMARY (ATE/RPE for systems with valid timestamps)")
    print("=" * 90)

    # ── Per-dataset breakdown ──
    for dataset in sorted(df["dataset"].unique()):
        ds = df[df["dataset"] == dataset]
        print(f"\n{'─' * 70}")
        print(f"  Dataset: {dataset}")
        print(f"{'─' * 70}")

        # Baseline
        base = ds[ds["overlay"] == "original"]
        if not base.empty:
            print(f"\n  {'SLAM':12s} {'ATE RMSE':>10s} {'RPE_t':>10s} {'RPE_r':>10s} {'Track%':>8s}  [Baseline]")
            for _, r in base.iterrows():
                print(f"  {r['slam']:12s} {r['ate_rmse']:10.4f}m {r['rpe_trans_rmse']:10.4f}m "
                      f"{r['rpe_rot_rmse']:10.2f}° {r['tracking_rate']:7.1%}")

        # Attack by SLAM
        attack = ds[ds["overlay"] != "original"]
        if not attack.empty:
            print(f"\n  {'SLAM':12s} {'ATE mean':>10s} {'ATE std':>10s} {'Track%':>8s}  [Attack avg]")
            for slam in sorted(attack["slam"].unique()):
                sub = attack[attack["slam"] == slam]
                ate_m = sub["ate_rmse"].mean()
                ate_s = sub["ate_rmse"].std()
                tr = sub["tracking_rate"].mean()
                print(f"  {slam:12s} {ate_m:10.4f}m {ate_s:10.4f}m {tr:7.1%}")

            # By intensity
            print(f"\n  {'Intensity':>10s} {'ATE RMSE':>10s} {'Track%':>8s}")
            for intensity in sorted(attack["intensity"].unique()):
                sub = attack[attack["intensity"] == intensity]
                print(f"  {'i='+str(intensity):>10s} {sub['ate_rmse'].mean():10.4f}m {sub['tracking_rate'].mean():7.1%}")

    # ── DSO tracking rate analysis (no ATE, but tracking rate is informative) ──
    dso_data = df_all[df_all["slam"] == "dso"]
    if not dso_data.empty:
        print(f"\n{'=' * 90}")
        print("DSO TRACKING RATE ANALYSIS (timestamps unavailable, ATE not computed)")
        print(f"{'=' * 90}")
        for dataset in sorted(dso_data["dataset"].unique()):
            ds = dso_data[dso_data["dataset"] == dataset]
            base = ds[ds["overlay"] == "original"]
            attack = ds[ds["overlay"] != "original"]
            base_tr = base["tracking_rate"].mean() if not base.empty else np.nan
            attack_tr = attack["tracking_rate"].mean() if not attack.empty else np.nan
            base_str = f"{base_tr:.1%}" if not np.isnan(base_tr) else "N/A"
            attack_str = f"{attack_tr:.1%}" if not np.isnan(attack_tr) else "N/A"

            if not attack.empty:
                by_int = attack.groupby("intensity")["tracking_rate"].mean()
                int_str = "  ".join(f"i{k:.1f}={v:.1%}" for k, v in by_int.items())
            else:
                int_str = ""

            print(f"  {dataset:40s}  base={base_str:>6s}  attack={attack_str:>6s}  {int_str}")

    # ── Overall summary ──
    print(f"\n{'=' * 90}")
    print("OVERALL ATTACK IMPACT (systems with ATE)")
    print(f"{'=' * 90}")
    print(f"  {'SLAM':12s} {'Base ATE':>10s} {'Atk ATE':>10s} {'Degradation':>12s} {'Base Track':>11s} {'Atk Track':>11s}")
    base_all = df[df["overlay"] == "original"]
    attack_all = df[df["overlay"] != "original"]
    for slam in sorted(df["slam"].unique()):
        b = base_all[base_all["slam"] == slam]["ate_rmse"].mean()
        a = attack_all[attack_all["slam"] == slam]["ate_rmse"].mean()
        bt = base_all[base_all["slam"] == slam]["tracking_rate"].mean()
        at = attack_all[attack_all["slam"] == slam]["tracking_rate"].mean()
        if not np.isnan(a):
            if not np.isnan(b) and b > 0:
                pct = f"{((a - b) / b * 100):+.1f}%"
            else:
                pct = "N/A"
            bt_str = f"{bt:.1%}" if not np.isnan(bt) else "N/A"
            print(f"  {slam:12s} {b:10.4f}m {a:10.4f}m {pct:>12s} {bt_str:>11s} {at:10.1%}")

    # ── TUM detailed table (for paper) ──
    tum = df[df["dataset"].str.contains("freiburg")]
    if not tum.empty:
        print(f"\n{'=' * 90}")
        print("TUM RGB-D DETAILED TABLE (for paper)")
        print(f"{'=' * 90}")
        print(f"  {'Dataset':30s} {'Overlay':>8s} {'Int':>5s} {'SLAM':>12s} "
              f"{'ATE RMSE':>10s} {'RPE_t':>10s} {'RPE_r':>8s} {'Track%':>8s}")
        for _, r in tum.sort_values(["dataset", "slam", "overlay", "intensity"]).iterrows():
            ov = str(r["overlay"])
            print(f"  {r['dataset']:30s} {ov:>8s} {r['intensity']:5.1f} {r['slam']:>12s} "
                  f"{r['ate_rmse']:10.4f}m {r['rpe_trans_rmse']:10.4f}m {r['rpe_rot_rmse']:8.2f}° "
                  f"{r['tracking_rate']:7.1%}")


if __name__ == "__main__":
    print("Starting evaluation...")
    warnings.filterwarnings("ignore")

    rows = run_evaluation()
    print(f"\nCollected {len(rows)} result entries.")

    df = pd.DataFrame(rows)
    df.to_csv(CSV_PATH, index=False)
    print(f"Saved to {CSV_PATH}")

    # filter ok rows for analysis
    df_ok = df[df["status"] == "ok"].copy()
    print(f"Valid entries: {len(df_ok)}")

    if not df_ok.empty:
        print_summary(df_ok, df)
        print("\nGenerating plots...")
        generate_plots(df_ok)
        print(f"Plots saved to {PLOT_DIR}/")

    print("\nDone.")
