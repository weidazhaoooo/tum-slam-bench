#!/bin/bash
#
# Download TUM RGB-D SLAM datasets
# Only downloads datasets that have ORB-SLAM3 association files
#
# Usage:
#   bash scripts/download_tum.sh          # download all
#   bash scripts/download_tum.sh fr1_desk # download specific one
#

BASE="${TUM_DATA_DIR:-/home/weida/v_slam_dataset/3_origin_running_video_tum_format_data}"
TUM_URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg"

# Dataset name → download URL mapping
declare -A DATASETS=(
    # Freiburg 1 (TUM1.yaml)
    ["rgbd_dataset_freiburg1_xyz"]="${TUM_URL}1/rgbd_dataset_freiburg1_xyz.tgz"
    ["rgbd_dataset_freiburg1_desk"]="${TUM_URL}1/rgbd_dataset_freiburg1_desk.tgz"
    ["rgbd_dataset_freiburg1_desk2"]="${TUM_URL}1/rgbd_dataset_freiburg1_desk2.tgz"
    ["rgbd_dataset_freiburg1_room"]="${TUM_URL}1/rgbd_dataset_freiburg1_room.tgz"

    # Freiburg 2 (TUM2.yaml)
    ["rgbd_dataset_freiburg2_xyz"]="${TUM_URL}2/rgbd_dataset_freiburg2_xyz.tgz"
    ["rgbd_dataset_freiburg2_desk"]="${TUM_URL}2/rgbd_dataset_freiburg2_desk.tgz"

    # Freiburg 3 (TUM3.yaml)
    ["rgbd_dataset_freiburg3_nstr_tex_near"]="${TUM_URL}3/rgbd_dataset_freiburg3_nostructure_texture_near_withloop.tgz"
    ["rgbd_dataset_freiburg3_str_tex_far"]="${TUM_URL}3/rgbd_dataset_freiburg3_structure_texture_far.tgz"
    ["rgbd_dataset_freiburg3_str_tex_near"]="${TUM_URL}3/rgbd_dataset_freiburg3_structure_texture_near.tgz"
    ["rgbd_dataset_freiburg3_long_office"]="${TUM_URL}3/rgbd_dataset_freiburg3_long_office_household.tgz"
)

# Sizes for reference
declare -A SIZES=(
    ["rgbd_dataset_freiburg1_xyz"]="0.47 GB"
    ["rgbd_dataset_freiburg1_desk"]="0.36 GB"
    ["rgbd_dataset_freiburg1_desk2"]="0.37 GB"
    ["rgbd_dataset_freiburg1_room"]="0.83 GB"
    ["rgbd_dataset_freiburg2_xyz"]="2.39 GB"
    ["rgbd_dataset_freiburg2_desk"]="2.01 GB"
    ["rgbd_dataset_freiburg3_nstr_tex_near"]="0.35 GB"
    ["rgbd_dataset_freiburg3_str_tex_far"]="0.44 GB"
    ["rgbd_dataset_freiburg3_str_tex_near"]="0.20 GB"
    ["rgbd_dataset_freiburg3_long_office"]="1.58 GB"
)

mkdir -p "$BASE"

download_dataset() {
    local name="$1"
    local url="${DATASETS[$name]}"
    local size="${SIZES[$name]:-unknown}"

    if [[ -d "$BASE/$name" ]]; then
        echo "[SKIP] $name already exists"
        return
    fi

    if [[ -z "$url" ]]; then
        echo "[ERROR] Unknown dataset: $name"
        return
    fi

    echo "[DOWNLOAD] $name ($size)..."
    local tgz="/tmp/${name}.tgz"
    wget -q --show-progress -O "$tgz" "$url"

    if [[ $? -ne 0 ]]; then
        echo "[ERROR] Failed to download $name"
        rm -f "$tgz"
        return
    fi

    echo "[EXTRACT] $name..."
    tar -xzf "$tgz" -C "$BASE/"
    rm -f "$tgz"

    # Some datasets extract with different names, rename if needed
    if [[ ! -d "$BASE/$name" ]]; then
        # Find the extracted directory
        local extracted=$(ls -dt "$BASE"/rgbd_dataset_freiburg* 2>/dev/null | head -1)
        if [[ -n "$extracted" && "$extracted" != "$BASE/$name" ]]; then
            mv "$extracted" "$BASE/$name"
        fi
    fi

    echo "[DONE] $name → $BASE/$name"
}

# Main
if [[ $# -gt 0 ]]; then
    # Download specific datasets
    for arg in "$@"; do
        # Allow short names
        case "$arg" in
            fr1_xyz)         download_dataset "rgbd_dataset_freiburg1_xyz" ;;
            fr1_desk)        download_dataset "rgbd_dataset_freiburg1_desk" ;;
            fr1_desk2)       download_dataset "rgbd_dataset_freiburg1_desk2" ;;
            fr1_room)        download_dataset "rgbd_dataset_freiburg1_room" ;;
            fr2_xyz)         download_dataset "rgbd_dataset_freiburg2_xyz" ;;
            fr2_desk)        download_dataset "rgbd_dataset_freiburg2_desk" ;;
            fr3_office)      download_dataset "rgbd_dataset_freiburg3_long_office" ;;
            all)
                for name in "${!DATASETS[@]}"; do
                    download_dataset "$name"
                done
                ;;
            *)               download_dataset "$arg" ;;
        esac
    done
else
    # Show available datasets
    echo "TUM RGB-D Dataset Downloader"
    echo "============================"
    echo ""
    echo "Available datasets:"
    echo ""
    printf "  %-45s %10s %s\n" "Name" "Size" "Status"
    printf "  %-45s %10s %s\n" "----" "----" "------"
    for name in $(echo "${!DATASETS[@]}" | tr ' ' '\n' | sort); do
        local_status="not downloaded"
        [[ -d "$BASE/$name" ]] && local_status="downloaded ✓"
        printf "  %-45s %10s %s\n" "$name" "${SIZES[$name]}" "$local_status"
    done
    echo ""
    echo "Usage:"
    echo "  bash scripts/download_tum.sh all              # download all (~8.6 GB)"
    echo "  bash scripts/download_tum.sh fr1_desk fr1_room  # download specific"
    echo "  bash scripts/download_tum.sh fr2_xyz            # single dataset"
    echo ""
    echo "Short names: fr1_xyz fr1_desk fr1_desk2 fr1_room fr2_xyz fr2_desk fr3_office"
fi
