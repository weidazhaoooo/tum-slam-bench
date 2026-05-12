#!/bin/bash
#
# Generate comparison videos for ALL ORB-SLAM3 results that have map/ and annotated/ but no comparison.mp4
#
# Usage: bash scripts/generate_all_videos.sh
#

RESULTS="${SLAM_RESULTS_DIR:-/home/weida/v_slam_dataset/5_results}"
COUNT=0
TOTAL=0

for dir in "$RESULTS"/*/orbslam3; do
    [[ ! -d "$dir/map" || ! -d "$dir/annotated" ]] && continue
    TOTAL=$((TOTAL + 1))

    [[ -f "$dir/comparison.mp4" ]] && continue

    MAP_COUNT=$(ls "$dir/map/" 2>/dev/null | wc -l)
    ANNOT_COUNT=$(ls "$dir/annotated/" 2>/dev/null | wc -l)
    [[ $MAP_COUNT -eq 0 || $ANNOT_COUNT -eq 0 ]] && continue

    RUN_NAME=$(basename "$(dirname "$dir")")
    COUNT=$((COUNT + 1))
    echo "[$COUNT] $RUN_NAME (map: $MAP_COUNT, annotated: $ANNOT_COUNT)"

    # map → mp4
    ffmpeg -y -framerate 30 -i "$dir/map/frame_%06d.png" \
        -c:v libx264 -pix_fmt yuv420p -crf 20 "$dir/map.mp4" 2>/dev/null

    # annotated → mp4
    ffmpeg -y -framerate 30 -i "$dir/annotated/frame_%06d.png" \
        -c:v libx264 -pix_fmt yuv420p -crf 20 "$dir/annotated.mp4" 2>/dev/null

    # side-by-side comparison
    ffmpeg -y -i "$dir/map.mp4" -i "$dir/annotated.mp4" \
        -filter_complex \
        "[0:v]scale=1024:768,drawtext=text='3D Map':fontsize=32:fontcolor=white:borderw=2:bordercolor=black:x=30:y=20[left];
         [1:v]scale=1024:768:force_original_aspect_ratio=decrease,pad=1024:768:(ow-iw)/2:(oh-ih)/2:color=black,drawtext=text='Feature Tracking':fontsize=32:fontcolor=white:borderw=2:bordercolor=black:x=30:y=20[right];
         [left][right]hstack=inputs=2[v]" \
        -map "[v]" -c:v libx264 -pix_fmt yuv420p -crf 20 -shortest \
        "$dir/comparison.mp4" 2>/dev/null

    SIZE=$(du -h "$dir/comparison.mp4" 2>/dev/null | cut -f1)
    echo "  → $dir/comparison.mp4 ($SIZE)"
done

echo ""
echo "Done! Generated $COUNT videos ($TOTAL total ORB-SLAM3 results)"
