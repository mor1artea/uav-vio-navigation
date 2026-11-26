#!/usr/bin/env bash
# Replays a simulated flight with our filter and prints the numbers. One command per variant:
#
#   sim/replay.sh ~/datasets/sim/straight_calm off    # midpoint linearization off (the default)
#   sim/replay.sh ~/datasets/sim/straight_calm on     # midpoint linearization on
#
# Output in <session>/vio_off or <session>/vio_on: trajectory.csv (vio_replay output) and summary.txt
# (filter log, ATE / RPE, consistency, signed scale). Run from anywhere; needs the built repository.

set -euo pipefail
[[ $# -eq 2 && ( "$2" == off || "$2" == on ) ]] || { echo "usage: $0 <session folder> off|on" >&2; exit 1; }
REPO="$(cd "$(dirname "$0")/.." && pwd)"
SESSION="$(cd "$1" && pwd)"
OUT="$SESSION/vio_$2"
mkdir -p "$OUT"

configs=(--config "$REPO/config/vio.yaml" --config "$REPO/config/replay_sim.yaml")
[[ "$2" == on ]] && configs+=(--config "$REPO/config/midpoint_on.yaml")

{
    echo "== vio_replay, midpoint $2"
    "$REPO/build/vio_replay" "$SESSION" --mode vio "${configs[@]}" --out "$OUT/trajectory.csv"
    echo "== metrics"
    python3 "$REPO/tools/metrics.py" "$OUT/trajectory.csv" "$SESSION"
    echo "== consistency"
    python3 "$REPO/tools/consistency.py" "$OUT/trajectory.csv" "$SESSION"
    echo "== signed scale (estimated / true distance; above 1: speed too high)"
    python3 "$REPO/tools/scale.py" "$OUT/trajectory.csv" "$SESSION"
} 2>&1 | tee "$OUT/summary.txt"
