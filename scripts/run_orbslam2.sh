#!/bin/bash
# Wrapper for ORB-SLAM2 that handles crashes gracefully
# Runs ORB-SLAM2 in a subprocess and collects whatever output it produces

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"
PROJ_B_DIR="$(dirname "$SCRIPT_DIR")"
EXTERNAL_DIR="${EXTERNAL_DIR:-$PROJ_B_DIR/external}"
ORBSLAM2_DIR="$EXTERNAL_DIR/ORB_SLAM2"
VOCAB="$ORBSLAM2_DIR/Vocabulary/ORBvoc.txt"

MODE="$1"     # rgbd or mono
YAML="$2"     # yaml config path
SEQ="$3"      # sequence path
ASSOC="$4"    # association file (for rgbd only)
OUTDIR="$5"   # output directory

mkdir -p "$OUTDIR"

export LD_LIBRARY_PATH="$ORBSLAM2_DIR/lib:$ORBSLAM2_DIR/Thirdparty/DBoW2/lib:$ORBSLAM2_DIR/Thirdparty/g2o/lib:$LD_LIBRARY_PATH"
unset ORB_SLAM2_VIEWER

cd "$ORBSLAM2_DIR"

if [[ "$MODE" == "mono" ]]; then
    ./Examples/Monocular/mono_tum "$VOCAB" "$YAML" "$SEQ" \
        > "$OUTDIR/orbslam2_output.txt" 2>&1
else
    ./Examples/RGB-D/rgbd_tum "$VOCAB" "$YAML" "$SEQ" "$ASSOC" \
        > "$OUTDIR/orbslam2_output.txt" 2>&1
fi

RC=$?

# Move any trajectory files produced
for f in CameraTrajectory.txt KeyFrameTrajectory.txt; do
    [[ -f "$ORBSLAM2_DIR/$f" ]] && mv "$ORBSLAM2_DIR/$f" "$OUTDIR/"
done

if [[ $RC -ne 0 ]]; then
    echo "ORB-SLAM2 exited with code $RC (crash expected due to g2o bug)"
fi

TRAJ_LINES=$(wc -l < "$OUTDIR/CameraTrajectory.txt" 2>/dev/null || echo 0)
echo "Trajectory: $TRAJ_LINES poses saved to $OUTDIR/"
