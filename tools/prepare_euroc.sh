#!/usr/bin/env bash
# Prepare EuRoC sequences that were downloaded by hand (ASL format, not rosbag).
#
#   tools/prepare_euroc.sh ~/Downloads/MH_01_easy.zip ...    # unpack zips into ~/datasets/euroc
#   tools/prepare_euroc.sh ~/datasets/euroc/MH_01_easy        # only check an unpacked folder
#   DEST=/data/euroc tools/prepare_euroc.sh file.zip          # other destination
#
# Result per sequence:  $DEST/<sequence>/mav0/{cam0,imu0,state_groundtruth_estimate0}
# Needed: cam0 (data.csv, data/*.png, sensor.yaml), imu0 (data.csv, sensor.yaml),
#         state_groundtruth_estimate0/data.csv (ground truth for the metrics).
# Only these and mav0/body.yaml are extracted. cam1, leica0, vicon0, pointcloud0 stay in the zip.

set -euo pipefail

dest="${DEST:-$HOME/datasets/euroc}"

if [[ $# -eq 0 ]]; then
    echo "usage: $0 <sequence.zip | unpacked sequence folder> ..." >&2
    exit 1
fi

unpack() {  # unpack <zip> <dir>: only cam0, imu0, ground truth and body.yaml
    # Python's zipfile drops ".." and absolute parts from member names, so nothing lands outside <dir>.
    python3 -I - "$1" "$2" <<'EOF'
import sys, zipfile
keep = ("mav0/cam0/", "mav0/imu0/", "mav0/state_groundtruth_estimate0/", "mav0/body.yaml")
with zipfile.ZipFile(sys.argv[1]) as z:
    members = [m for m in z.namelist()
               if any(k in m for k in keep) and not m.endswith(".DS_Store")]
    z.extractall(sys.argv[2], members)
print(f"extracted {len(members)} of {len(z.namelist())} entries")
EOF
}

check() {  # check <sequence folder>, prints a short summary or fails
    local seq="$1" name
    name="$(basename "$seq")"
    local need=(
        mav0/cam0/data.csv mav0/cam0/sensor.yaml
        mav0/imu0/data.csv mav0/imu0/sensor.yaml
        mav0/state_groundtruth_estimate0/data.csv
    )
    for f in "${need[@]}"; do
        [[ -f "$seq/$f" ]] || { echo "error: $name has no $f" >&2; return 1; }
    done
    local rows pngs
    rows=$(grep -vc '^#' "$seq/mav0/cam0/data.csv" || true)
    pngs=$(find "$seq/mav0/cam0/data" -maxdepth 1 -name '*.png' | wc -l)
    if [[ "$rows" -ne "$pngs" ]]; then
        echo "error: $name cam0 lists $rows images but has $pngs png files" >&2
        return 1
    fi
    echo "ok $name: $pngs images, $(grep -vc '^#' "$seq/mav0/imu0/data.csv") imu rows," \
         "$(grep -vc '^#' "$seq/mav0/state_groundtruth_estimate0/data.csv") ground truth rows"
}

status=0
for arg in "$@"; do
    if [[ -d "$arg" ]]; then
        check "$arg" || status=1
        continue
    fi
    if [[ ! -f "$arg" || "$arg" != *.zip ]]; then
        echo "error: $arg is neither a folder nor a .zip file" >&2
        status=1
        continue
    fi

    name="$(basename "$arg" .zip)"
    out="$dest/$name"
    if [[ -d "$out/mav0" ]]; then
        echo "skip unpacking $name (already in $out)"
    else
        echo "unpack $name -> $out"
        mkdir -p "$out"
        unpack "$arg" "$out"
        # Some archives contain the sequence folder itself, move mav0 up if needed.
        if [[ ! -d "$out/mav0" ]]; then
            inner="$(find "$out" -maxdepth 3 -type d -name mav0 | head -n 1)"
            [[ -n "$inner" ]] && mv "$inner" "$out/mav0"
        fi
    fi
    check "$out" || status=1
done

exit $status
