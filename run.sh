#!/bin/bash
#
# Proj B — TUM SLAM Benchmark
#
# Runs one or more SLAM/MDE systems on a TUM-format dataset and produces
# trajectories + standardized result indexes. Knows nothing about overlay
# attacks — that's proj_a's job.
#
# Usage:
#   bash run.sh --dataset freiburg1_xyz --slam orbslam3
#   bash run.sh --dataset /path/to/tum_dataset --slam orbslam3,dso,openvslam
#   bash run.sh --dataset carla_sim --slam orbslam3 --mode mono
#

set -e

# ==================== Paths ====================
# SLAM binaries + proj_b's own scripts resolve relative to this file, so the
# repo is relocatable. Data lake (origin datasets + results) still lives at
# the workspace root for now (BASE), shared with proj_a.
PROJ_B_DIR="$(dirname "$(readlink -f "$0")")"
EXTERNAL_DIR="$PROJ_B_DIR/external"
SCRIPTS_DIR="$PROJ_B_DIR/scripts"

# Data lake locations — env-var overridable so the same script works on
# any machine. Defaults match the current development workspace.
BASE="${VSLAM_BASE:-/home/weida/v_slam_dataset}"
ORIGIN_DATA="${TUM_DATA_DIR:-$BASE/3_origin_running_video_tum_format_data}"
RESULTS="${SLAM_RESULTS_DIR:-$BASE/5_results}"

ORBSLAM3_DIR="$EXTERNAL_DIR/ORB_SLAM3"
ORBSLAM2_DIR="$EXTERNAL_DIR/ORB_SLAM2"
DSO_DIR="$EXTERNAL_DIR/DSO"
DSO_BIN="$DSO_DIR/build/bin/dso_dataset_nogui"
OPENVSLAM_DIR="$EXTERNAL_DIR/OpenVSLAM"
OPENVSLAM_BIN="$OPENVSLAM_DIR/bin/run_tum_rgbd_slam"
OPENVSLAM_VOCAB="$OPENVSLAM_DIR/orb_vocab.fbow"
OPENVSLAM_LOCAL="$OPENVSLAM_DIR/local_install"
RTABMAP_BIN="rtabmap-rgbd_dataset"
MIDAS_DIR="$EXTERNAL_DIR/MiDaS"
DAV2_DIR="$EXTERNAL_DIR/DepthAnythingV2"
MDE_CONDA="sam3"
SYNC_SCRIPT="$SCRIPTS_DIR/prepare_rtabmap_sync.py"
VIS_SCRIPT="$SCRIPTS_DIR/generate_visualizations.py"
MAPPOINTS_SCRIPT="$SCRIPTS_DIR/mappoints_to_ply.py"
STANDARDIZE_SCRIPT="$SCRIPTS_DIR/standardize_results.py"
PREPARE_DSO_SCRIPT="$SCRIPTS_DIR/prepare_dso_input.py"

# ==================== Defaults ====================
DATASET=""
MODE="auto"         # auto / rgbd / mono
SLAM_SYSTEMS="orbslam3"
RUN_NAME=""
SKIP_SLAM=false
SKIP_VIDEO=false
NO_VIEWER=false

# ==================== Help ====================
show_help() {
    cat << 'EOF'
Proj B — TUM SLAM Benchmark
============================

Usage: bash run.sh [options]

Required:
  --dataset NAME|PATH   TUM-format dataset directory. Either an absolute path,
                        or one of these short names:
                          freiburg1_xyz  freiburg1_desk  freiburg1_desk2  freiburg1_room
                          freiburg2_xyz  freiburg2_desk
                          freiburg3_office
                          carla_sim

SLAM Systems:
  --slam SYSTEMS        Comma-separated: orbslam3,orbslam2,dso,openvslam,rtabmap,midas,dav2
                        "all" expands to all of the above. Default: orbslam3
  --mode MODE           auto (default) / rgbd / mono

Output:
  --run-name NAME       Result subdir name (default: basename of dataset path)

Skip Stages:
  --skip-slam           Don't re-run SLAM (only do video/standardize on existing results)
  --skip-video          Skip video generation
  --no-viewer           Fast ORB-SLAM3, no frame saving

Examples:
  bash run.sh --dataset freiburg1_xyz --slam orbslam3
  bash run.sh --dataset /path/to/dataset --slam orbslam3,dso,openvslam
  bash run.sh --dataset carla_sim --slam orbslam3 --mode mono
EOF
    exit 0
}

# ==================== Parse Args ====================
while [[ $# -gt 0 ]]; do
    case $1 in
        --help|-h) show_help ;;
        --dataset) DATASET="$2"; shift 2 ;;
        --slam)
            if [[ "$2" == "all" ]]; then
                SLAM_SYSTEMS="orbslam3,orbslam2,dso,openvslam,rtabmap,midas,dav2"
            else
                SLAM_SYSTEMS="$2"
            fi
            shift 2 ;;
        --mode) MODE="$2"; shift 2 ;;
        --run-name) RUN_NAME="$2"; shift 2 ;;
        --skip-slam) SKIP_SLAM=true; shift ;;
        --skip-video) SKIP_VIDEO=true; shift ;;
        --no-viewer) NO_VIEWER=true; shift ;;
        *) echo "Unknown: $1"; show_help ;;
    esac
done

[[ -z "$DATASET" ]] && echo "Error: --dataset required" && exit 1

# ==================== Resolve Names ====================
resolve_dataset_name() {
    case "$1" in
        freiburg1_xyz|fr1_xyz)     echo "rgbd_dataset_freiburg1_xyz" ;;
        freiburg1_desk|fr1_desk)   echo "rgbd_dataset_freiburg1_desk" ;;
        freiburg1_desk2|fr1_desk2) echo "rgbd_dataset_freiburg1_desk2" ;;
        freiburg1_room|fr1_room)   echo "rgbd_dataset_freiburg1_room" ;;
        freiburg2_xyz|fr2_xyz)     echo "rgbd_dataset_freiburg2_xyz" ;;
        freiburg2_desk|fr2_desk)   echo "rgbd_dataset_freiburg2_desk" ;;
        freiburg3_office|fr3_office) echo "rgbd_dataset_freiburg3_long_office" ;;
        carla_sim|carla)           echo "carla_sim_original" ;;
        carla_*)                   echo "$1" ;;
        *)                         echo "$1" ;;
    esac
}

if [[ "$DATASET" == /* ]]; then
    ATTACKED="$DATASET"
    FULL_DATASET=$(basename "$DATASET")
else
    FULL_DATASET=$(resolve_dataset_name "$DATASET")
    ATTACKED="$ORIGIN_DATA/$FULL_DATASET"
fi
[[ ! -d "$ATTACKED" ]] && echo "Error: dataset not found: $ATTACKED" && exit 1

[[ -z "$RUN_NAME" ]] && RUN_NAME="$(basename "$ATTACKED")"
SAVE_DIR="$RESULTS/$RUN_NAME"

resolve_mode() {
    if [[ "$2" != "auto" ]]; then echo "$2"; return; fi
    if [[ "$1" == *carla* ]]; then echo "mono"; else echo "rgbd"; fi
}
SLAM_MODE=$(resolve_mode "$FULL_DATASET" "$MODE")

resolve_yaml() {
    local d="$1" m="$2"
    if [[ "$m" == "mono" ]]; then
        [[ "$d" == *carla* ]] && echo "$ORBSLAM3_DIR/Examples/Monocular/CARLA.yaml" && return
        echo "$ORBSLAM3_DIR/Examples/Monocular/TUM1.yaml"
    else
        [[ "$d" == *freiburg1* ]] && echo "$ORBSLAM3_DIR/Examples/RGB-D/TUM1.yaml" && return
        [[ "$d" == *freiburg2* ]] && echo "$ORBSLAM3_DIR/Examples/RGB-D/TUM2.yaml" && return
        [[ "$d" == *freiburg3* ]] && echo "$ORBSLAM3_DIR/Examples/RGB-D/TUM3.yaml" && return
        echo "$ORBSLAM3_DIR/Examples/RGB-D/TUM1.yaml"
    fi
}

resolve_assoc() {
    local p="$1"
    [[ -f "$p/associations.txt" ]] && echo "$p/associations.txt" && return
    local d="$ORBSLAM3_DIR/Examples/RGB-D/associations"
    local n=$(basename "$p")
    case "$n" in
        *freiburg1_xyz*)  echo "$d/fr1_xyz.txt" ;;
        *freiburg1_desk*) echo "$d/fr1_desk.txt" ;;
        *freiburg1_room*) echo "$d/fr1_room.txt" ;;
        *freiburg2_xyz*)  echo "$d/fr2_xyz.txt" ;;
        *freiburg2_desk*) echo "$d/fr2_desk.txt" ;;
        *)                echo "$d/fr1_xyz.txt" ;;
    esac
}

resolve_orbslam2_yaml() {
    local d="$1" m="$2"
    if [[ "$m" == "mono" ]]; then
        [[ "$d" == *freiburg1* ]] && echo "$ORBSLAM2_DIR/Examples/Monocular/TUM1.yaml" && return
        [[ "$d" == *freiburg2* ]] && echo "$ORBSLAM2_DIR/Examples/Monocular/TUM2.yaml" && return
        [[ "$d" == *freiburg3* ]] && echo "$ORBSLAM2_DIR/Examples/Monocular/TUM3.yaml" && return
        echo "$ORBSLAM2_DIR/Examples/Monocular/TUM1.yaml"
    else
        [[ "$d" == *freiburg1* ]] && echo "$ORBSLAM2_DIR/Examples/RGB-D/TUM1.yaml" && return
        [[ "$d" == *freiburg2* ]] && echo "$ORBSLAM2_DIR/Examples/RGB-D/TUM2.yaml" && return
        [[ "$d" == *freiburg3* ]] && echo "$ORBSLAM2_DIR/Examples/RGB-D/TUM3.yaml" && return
        echo "$ORBSLAM2_DIR/Examples/RGB-D/TUM1.yaml"
    fi
}

resolve_orbslam2_assoc() {
    local p="$1"
    [[ -f "$p/associations.txt" ]] && echo "$p/associations.txt" && return
    local d="$ORBSLAM2_DIR/Examples/RGB-D/associations"
    local n=$(basename "$p")
    case "$n" in
        *freiburg1_xyz*)  echo "$d/fr1_xyz.txt" ;;
        *freiburg1_desk*) echo "$d/fr1_desk.txt" ;;
        *freiburg1_room*) echo "$d/fr1_room.txt" ;;
        *freiburg2_xyz*)  echo "$d/fr2_xyz.txt" ;;
        *freiburg2_desk*) echo "$d/fr2_desk.txt" ;;
        *)                echo "$d/fr1_xyz.txt" ;;
    esac
}

resolve_openvslam_config() {
    local d="$1" m="$2"
    local cfg_dir="$OPENVSLAM_DIR/example/tum_rgbd"
    if [[ "$m" == "mono" ]]; then
        [[ "$d" == *freiburg1* ]] && echo "$cfg_dir/TUM_RGBD_mono_1.yaml" && return
        [[ "$d" == *freiburg2* ]] && echo "$cfg_dir/TUM_RGBD_mono_2.yaml" && return
        [[ "$d" == *freiburg3* ]] && echo "$cfg_dir/TUM_RGBD_mono_3.yaml" && return
        echo "$cfg_dir/TUM_RGBD_mono_1.yaml"
    else
        [[ "$d" == *freiburg1* ]] && echo "$cfg_dir/TUM_RGBD_rgbd_1.yaml" && return
        [[ "$d" == *freiburg2* ]] && echo "$cfg_dir/TUM_RGBD_rgbd_2.yaml" && return
        [[ "$d" == *freiburg3* ]] && echo "$cfg_dir/TUM_RGBD_rgbd_3.yaml" && return
        echo "$cfg_dir/TUM_RGBD_rgbd_1.yaml"
    fi
}

resolve_dso_calib() {
    local d="$1"
    [[ "$d" == *freiburg1* ]] && echo "$DSO_DIR/configs/tum_freiburg1.txt" && return
    [[ "$d" == *freiburg2* ]] && echo "$DSO_DIR/configs/tum_freiburg2.txt" && return
    [[ "$d" == *freiburg3* ]] && echo "$DSO_DIR/configs/tum_freiburg3.txt" && return
    echo "$DSO_DIR/configs/tum_freiburg1.txt"
}

# ==================== Logging ====================
LOGFILE="$RESULTS/run_log_$(date +%Y%m%d_%H%M%S).txt"
mkdir -p "$RESULTS"

log() {
    echo "$1"
    echo "[$(date '+%H:%M:%S')] $1" >> "$LOGFILE"
}

log_cmd() {
    echo "  CMD: $*" >> "$LOGFILE"
}

# ==================== Print Config ====================
log "=============================================="
log " TUM SLAM Benchmark"
log "=============================================="
log " Date:    $(date)"
log " Dataset: $ATTACKED ($SLAM_MODE)"
log " SLAM:    $SLAM_SYSTEMS"
log " Output:  $SAVE_DIR"
log " Log:     $LOGFILE"
log "=============================================="

export DISPLAY=:1

# ==================== Step 1: Run SLAM ====================
if [[ "$SKIP_SLAM" == false ]]; then

    # --- ORB-SLAM3 ---
    if [[ "$SLAM_SYSTEMS" == *orbslam3* ]]; then
        ORBSLAM_SAVE="$SAVE_DIR/orbslam3"
        mkdir -p "$ORBSLAM_SAVE"
        log "[Step 1] Running ORB-SLAM3 ($SLAM_MODE)..."

        cd "$ORBSLAM3_DIR"
        export LD_LIBRARY_PATH="$ORBSLAM3_DIR/lib:$ORBSLAM3_DIR/Thirdparty/DBoW2/lib:$ORBSLAM3_DIR/Thirdparty/g2o/lib:$LD_LIBRARY_PATH"

        [[ "$NO_VIEWER" == true ]] && unset SLAM_SAVE_DIR || export SLAM_SAVE_DIR="$ORBSLAM_SAVE"

        YAML=$(resolve_yaml "$FULL_DATASET" "$SLAM_MODE")

        # ORB-SLAM3 sometimes aborts in its destructor ("pure virtual method
        # called") *after* trajectories are saved — disable -e around the
        # call so we still recover the saved files.
        set +e
        if [[ "$SLAM_MODE" == "mono" ]]; then
            SLAM_CMD="./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt $YAML $ATTACKED"
            log_cmd "$SLAM_CMD"
            ./Examples/Monocular/mono_tum \
                Vocabulary/ORBvoc.txt "$YAML" "$ATTACKED"
        else
            ASSOC=$(resolve_assoc "$ATTACKED")
            SLAM_CMD="./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt $YAML $ATTACKED $ASSOC"
            log_cmd "$SLAM_CMD"
            ./Examples/RGB-D/rgbd_tum \
                Vocabulary/ORBvoc.txt "$YAML" "$ATTACKED" "$ASSOC"
        fi
        ORBSLAM_RC=$?
        set -e

        for f in CameraTrajectory.txt KeyFrameTrajectory.txt MapPoints.txt; do
            [[ -f "$f" ]] && mv "$f" "$ORBSLAM_SAVE/"
        done

        if [[ $ORBSLAM_RC -ne 0 ]]; then
            log "  (ORB-SLAM3 exit=$ORBSLAM_RC; saved files: $(ls "$ORBSLAM_SAVE/" 2>/dev/null | grep -E 'Trajectory|MapPoints' | tr '\n' ' '))"
        fi

        if [[ -f "$ORBSLAM_SAVE/MapPoints.txt" ]]; then
            python3 "$MAPPOINTS_SCRIPT" "$ORBSLAM_SAVE/MapPoints.txt" "$ORBSLAM_SAVE/MapPoints.ply" \
                2>&1 | while read line; do log "  $line"; done
        fi

        log "[Step 1] ORB-SLAM3 done. map: $(ls "$ORBSLAM_SAVE/map/" 2>/dev/null | wc -l) | annotated: $(ls "$ORBSLAM_SAVE/annotated/" 2>/dev/null | wc -l)"
    fi

    # --- ORB-SLAM2 ---
    if [[ "$SLAM_SYSTEMS" == *orbslam2* ]]; then
        OS2_SAVE="$SAVE_DIR/orbslam2"
        mkdir -p "$OS2_SAVE"
        log "[Step 1] Running ORB-SLAM2 ($SLAM_MODE)..."

        cd "$ORBSLAM2_DIR"
        export LD_LIBRARY_PATH="$ORBSLAM2_DIR/lib:$ORBSLAM2_DIR/Thirdparty/DBoW2/lib:$ORBSLAM2_DIR/Thirdparty/g2o/lib:$LD_LIBRARY_PATH"

        OS2_YAML=$(resolve_orbslam2_yaml "$FULL_DATASET" "$SLAM_MODE")

        if [[ "$SLAM_MODE" == "mono" ]]; then
            log_cmd "./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt $OS2_YAML $ATTACKED"
            ./Examples/Monocular/mono_tum \
                Vocabulary/ORBvoc.txt "$OS2_YAML" "$ATTACKED" \
                2>&1 | tee "$OS2_SAVE/orbslam2_output.txt" | tail -5
        else
            OS2_ASSOC=$(resolve_orbslam2_assoc "$ATTACKED")
            log_cmd "./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt $OS2_YAML $ATTACKED $OS2_ASSOC"
            ./Examples/RGB-D/rgbd_tum \
                Vocabulary/ORBvoc.txt "$OS2_YAML" "$ATTACKED" "$OS2_ASSOC" \
                2>&1 | tee "$OS2_SAVE/orbslam2_output.txt" | tail -5
        fi

        for f in CameraTrajectory.txt KeyFrameTrajectory.txt; do
            [[ -f "$f" ]] && mv "$f" "$OS2_SAVE/"
        done

        TRAJ_LINES=$(wc -l < "$OS2_SAVE/CameraTrajectory.txt" 2>/dev/null || echo 0)
        log "[Step 1] ORB-SLAM2 done. Trajectory: $TRAJ_LINES poses"
    fi

    # --- DSO ---
    if [[ "$SLAM_SYSTEMS" == *dso* ]]; then
        DSO_SAVE="$SAVE_DIR/dso"
        mkdir -p "$DSO_SAVE"
        log "[Step 1] Running DSO (monocular direct)..."

        # DSO has two requirements our raw dataset layout doesn't meet:
        #   1. It reads frames via std::sort(files) over `rgb/` — for
        #      CARLA that interleaves 10.800000.png between 108.x files
        #      and scrambles temporal order. TUM filenames happen to
        #      sort correctly, but we unify both paths anyway.
        #   2. It reads a sibling `times.txt` (`<id> <stamp> <expo>`);
        #      without it every pose in result.txt gets timestamp=0 and
        #      ATE/RPE evaluation silently fails.
        # Fix: build a dso_input/rgb/ symlink farm with zero-padded
        # filenames (preserving temporal order under std::sort) plus a
        # matching times.txt, and hand that to DSO.
        DSO_INPUT="$ATTACKED/dso_input"
        python3 "$PREPARE_DSO_SCRIPT" "$ATTACKED" "$DSO_INPUT" \
            2>&1 | tee -a "$DSO_SAVE/dso_output.txt"
        DSO_N=$(wc -l < "$DSO_INPUT/times.txt" 2>/dev/null || echo 0)
        log "[Step 1] DSO input prepared ($DSO_N frames at $DSO_INPUT)"

        DSO_CALIB=$(resolve_dso_calib "$FULL_DATASET")
        DSO_CMD="$DSO_BIN files=$DSO_INPUT/rgb calib=$DSO_CALIB mode=1 nogui=1 quiet=1"
        log_cmd "$DSO_CMD"

        cd "$DSO_DIR"
        "$DSO_BIN" \
            "files=$DSO_INPUT/rgb" \
            "calib=$DSO_CALIB" \
            mode=1 \
            nogui=1 \
            quiet=1 2>&1 | tee -a "$DSO_SAVE/dso_output.txt" | tail -5

        [[ -f result.txt ]] && mv result.txt "$DSO_SAVE/CameraTrajectory.txt"

        TRAJ_LINES=$(wc -l < "$DSO_SAVE/CameraTrajectory.txt" 2>/dev/null || echo 0)
        log "[Step 1] DSO done. Trajectory: $TRAJ_LINES keyframes"
    fi

    # --- OpenVSLAM (stella_vslam) ---
    if [[ "$SLAM_SYSTEMS" == *openvslam* ]]; then
        OV_SAVE="$SAVE_DIR/openvslam"
        mkdir -p "$OV_SAVE"
        log "[Step 1] Running OpenVSLAM (stella_vslam, $SLAM_MODE)..."

        export LD_LIBRARY_PATH="$OPENVSLAM_LOCAL/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

        OV_CONFIG=$(resolve_openvslam_config "$FULL_DATASET" "$SLAM_MODE")
        OV_CMD="$OPENVSLAM_BIN -v $OPENVSLAM_VOCAB -d $ATTACKED -c $OV_CONFIG --no-sleep --auto-term --viewer none --eval-log-dir $OV_SAVE"
        log_cmd "$OV_CMD"

        "$OPENVSLAM_BIN" \
            -v "$OPENVSLAM_VOCAB" \
            -d "$ATTACKED" \
            -c "$OV_CONFIG" \
            --no-sleep \
            --auto-term \
            --viewer none \
            --eval-log-dir "$OV_SAVE" 2>&1 | tee "$OV_SAVE/openvslam_output.txt" | tail -5

        [[ -f "$OV_SAVE/frame_trajectory.txt" ]] && cp "$OV_SAVE/frame_trajectory.txt" "$OV_SAVE/CameraTrajectory.txt"

        TRAJ_LINES=$(wc -l < "$OV_SAVE/frame_trajectory.txt" 2>/dev/null || echo 0)
        log "[Step 1] OpenVSLAM done. Trajectory: $TRAJ_LINES frames"
    fi

    # --- RTAB-Map ---
    if [[ "$SLAM_SYSTEMS" == *rtabmap* ]]; then
        RT_SAVE="$SAVE_DIR/rtabmap"
        mkdir -p "$RT_SAVE"
        log "[Step 1] Running RTAB-Map..."

        source /opt/ros/jazzy/setup.bash 2>/dev/null

        python3 "$SYNC_SCRIPT" "$ATTACKED" 2>&1 | while read line; do log "[RTAB-Map] $line"; done

        RT_CMD="$RTABMAP_BIN --quiet --Vis/MaxFeatures 1000 --output $RT_SAVE --output_name rtabmap $ATTACKED"
        log_cmd "$RT_CMD"

        "$RTABMAP_BIN" \
            --quiet \
            --Vis/MaxFeatures 1000 \
            --output "$RT_SAVE" \
            --output_name rtabmap \
            "$ATTACKED" 2>&1 | tee "$RT_SAVE/rtabmap_output.txt" | tail -5

        [[ -f "$RT_SAVE/rtabmap_poses.txt" ]] && cp "$RT_SAVE/rtabmap_poses.txt" "$RT_SAVE/CameraTrajectory.txt"

        TRAJ_LINES=$(wc -l < "$RT_SAVE/rtabmap_poses.txt" 2>/dev/null || echo 0)
        RMSE=$(grep "translational_rmse" "$RT_SAVE/rtabmap_output.txt" 2>/dev/null | awk '{print $NF}' || echo "N/A")
        log "[Step 1] RTAB-Map done. Trajectory: $TRAJ_LINES frames, RMSE: $RMSE"
    fi

    # --- MiDaS (Monocular Depth Estimation) ---
    if [[ "$SLAM_SYSTEMS" == *midas* ]]; then
        MIDAS_SAVE="$SAVE_DIR/midas"
        mkdir -p "$MIDAS_SAVE"
        log "[Step 1] Running MiDaS MDE..."

        MIDAS_CMD="conda run -n $MDE_CONDA python $MIDAS_DIR/run_midas.py $ATTACKED $MIDAS_SAVE --model MiDaS_small --compare-gt"
        log_cmd "$MIDAS_CMD"

        conda run -n "$MDE_CONDA" python "$MIDAS_DIR/run_midas.py" \
            "$ATTACKED" "$MIDAS_SAVE" \
            --model MiDaS_small \
            --compare-gt 2>&1 | tee "$MIDAS_SAVE/midas_output.txt" | tail -15

        log "[Step 1] MiDaS done -> $MIDAS_SAVE"
    fi

    # --- Depth Anything V2 (Monocular Depth Estimation) ---
    if [[ "$SLAM_SYSTEMS" == *dav2* ]]; then
        DAV2_SAVE="$SAVE_DIR/depth_anything_v2"
        mkdir -p "$DAV2_SAVE"
        log "[Step 1] Running Depth Anything V2 MDE..."

        DAV2_CMD="conda run -n $MDE_CONDA python $DAV2_DIR/run_dav2.py $ATTACKED $DAV2_SAVE --model small --compare-gt"
        log_cmd "$DAV2_CMD"

        conda run -n "$MDE_CONDA" python "$DAV2_DIR/run_dav2.py" \
            "$ATTACKED" "$DAV2_SAVE" \
            --model small \
            --compare-gt 2>&1 | tee "$DAV2_SAVE/dav2_output.txt" | tail -15

        log "[Step 1] Depth Anything V2 done -> $DAV2_SAVE"
    fi

else
    log "[Step 1] Skipped (--skip-slam)"
fi

# ==================== Step 2: ORB-SLAM3 comparison videos ====================
if [[ "$SKIP_VIDEO" == false && "$NO_VIEWER" == false && "$SLAM_SYSTEMS" == *orbslam3* ]]; then
    ORBSLAM_SAVE="$SAVE_DIR/orbslam3"
    if [[ -d "$ORBSLAM_SAVE/map" && -d "$ORBSLAM_SAVE/annotated" ]]; then
        log "[Step 2] Creating comparison videos..."

        log_cmd "ffmpeg map -> $ORBSLAM_SAVE/map.mp4"
        ffmpeg -y -framerate 30 -i "$ORBSLAM_SAVE/map/frame_%06d.png" \
            -c:v libx264 -pix_fmt yuv420p -crf 20 "$ORBSLAM_SAVE/map.mp4" 2>/dev/null

        log_cmd "ffmpeg annotated -> $ORBSLAM_SAVE/annotated.mp4"
        ffmpeg -y -framerate 30 -i "$ORBSLAM_SAVE/annotated/frame_%06d.png" \
            -c:v libx264 -pix_fmt yuv420p -crf 20 "$ORBSLAM_SAVE/annotated.mp4" 2>/dev/null

        log_cmd "ffmpeg hstack -> $ORBSLAM_SAVE/comparison.mp4"
        ffmpeg -y -i "$ORBSLAM_SAVE/map.mp4" -i "$ORBSLAM_SAVE/annotated.mp4" \
            -filter_complex \
            "[0:v]scale=1024:768,drawtext=text='3D Map':fontsize=32:fontcolor=white:borderw=2:bordercolor=black:x=30:y=20[left];
             [1:v]scale=1024:768:force_original_aspect_ratio=decrease,pad=1024:768:(ow-iw)/2:(oh-ih)/2:color=black,drawtext=text='Feature Tracking':fontsize=32:fontcolor=white:borderw=2:bordercolor=black:x=30:y=20[right];
             [left][right]hstack=inputs=2[v]" \
            -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 20 -shortest \
            "$ORBSLAM_SAVE/comparison.mp4" 2>/dev/null

        log "[Step 2] Done -> $ORBSLAM_SAVE/comparison.mp4 ($(du -h "$ORBSLAM_SAVE/comparison.mp4" | cut -f1))"
    else
        log "[Step 2] Skipped (no map/ or annotated/ frames)"
    fi
else
    log "[Step 2] Skipped"
fi

# ==================== Step 3: Visualization for non-ORB-SLAM3 ====================
if [[ "$SKIP_VIDEO" == false ]]; then
    for slam_sub in dso openvslam rtabmap; do
        if [[ "$SLAM_SYSTEMS" == *$slam_sub* && -d "$SAVE_DIR/$slam_sub" ]]; then
            if [[ ! -f "$SAVE_DIR/$slam_sub/slam_visualization.mp4" ]]; then
                log "[Step 3] Generating visualization for $slam_sub..."
                log_cmd "python3 $VIS_SCRIPT $SAVE_DIR"
                python3 "$VIS_SCRIPT" "$SAVE_DIR" 2>&1 | while read line; do log "  $line"; done
                break  # VIS_SCRIPT processes all slam_subs in one call
            fi
        fi
    done
fi

# ==================== Summary ====================
log ""
log "Results for $RUN_NAME:"
if [[ "$SLAM_SYSTEMS" == *orbslam3* && -d "$SAVE_DIR/orbslam3" ]]; then
    log "  ORB-SLAM3:  $SAVE_DIR/orbslam3/"
fi
if [[ "$SLAM_SYSTEMS" == *dso* && -d "$SAVE_DIR/dso" ]]; then
    log "  DSO:        $SAVE_DIR/dso/"
fi
if [[ "$SLAM_SYSTEMS" == *openvslam* && -d "$SAVE_DIR/openvslam" ]]; then
    log "  OpenVSLAM:  $SAVE_DIR/openvslam/"
fi
if [[ "$SLAM_SYSTEMS" == *rtabmap* && -d "$SAVE_DIR/rtabmap" ]]; then
    log "  RTAB-Map:   $SAVE_DIR/rtabmap/"
fi
if [[ "$SLAM_SYSTEMS" == *midas* && -d "$SAVE_DIR/midas" ]]; then
    log "  MiDaS:      $SAVE_DIR/midas/"
fi
if [[ "$SLAM_SYSTEMS" == *dav2* && -d "$SAVE_DIR/depth_anything_v2" ]]; then
    log "  DAv2:       $SAVE_DIR/depth_anything_v2/"
fi

# ==================== Step 4: Standardize & index outputs ====================
if command -v python3 >/dev/null 2>&1 && [[ -f "$STANDARDIZE_SCRIPT" ]]; then
    log "[Step 4] Standardizing outputs and updating results index..."
    python3 "$STANDARDIZE_SCRIPT" --save-dir "$SAVE_DIR" --run-name "$RUN_NAME" --results-root "$RESULTS" 2>&1 | while read line; do log "  $line"; done || log "[Step 4] standardizer returned non-zero"
else
    log "[Step 4] Skipped standardization (python3 or script missing)"
fi

log ""
log "=============================================="
log " Done. Results in: $SAVE_DIR/"
log " Log: $LOGFILE"
log "=============================================="
