#!/usr/bin/env bash
# Starts Gazebo with the world of sim/make_world.py and ArduPilot SITL. Ctrl-C stops both.
#
#   sim/run_sim.sh ~/sim/world_line          # without a window
#   sim/run_sim.sh ~/sim/world_line --gui    # with the Gazebo window
#
# Then, in a second terminal: sim/fly_record.py ...
# Logs: <world>/gazebo.log, <world>/sitl/sitl.log

set -euo pipefail

SIM_DIR="${SIM_DIR:-$HOME/sim}"
[[ $# -ge 1 ]] || { echo "usage: $0 <world folder> [--gui]" >&2; exit 1; }
WORLD="$(cd "$1" && pwd)"
AP="$SIM_DIR/ardupilot"

for f in "$WORLD/world.sdf" "$AP/build/sitl/bin/arducopter" "$SIM_DIR/ardupilot_gazebo/build"; do
    [[ -e "$f" ]] || { echo "error: $f is missing (sim/setup.sh, sim/make_world.py)" >&2; exit 1; }
done

export GZ_SIM_SYSTEM_PLUGIN_PATH="$SIM_DIR/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$WORLD/models:$SIM_DIR/ardupilot_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

pids=()
stop() { kill "${pids[@]}" 2>/dev/null || true; wait 2>/dev/null || true; }
trap stop EXIT INT TERM

if [[ "${2:-}" == "--gui" ]]; then
    gz sim -r -v 3 "$WORLD/world.sdf" > "$WORLD/gazebo.log" 2>&1 &
else
    gz sim -r -s --headless-rendering -v 3 "$WORLD/world.sdf" > "$WORLD/gazebo.log" 2>&1 &
fi
pids+=($!)

mkdir -p "$WORLD/sitl"
cd "$WORLD/sitl"   # SITL writes its parameter storage and logs here
# copter.parm as sim_vehicle.py uses it; our own frame file instead of gazebo-iris.parm (no sonar, no
# precision landing). --model JSON: ardupilot_gazebo talks to SITL through the JSON interface.
REPO="$(cd "$(dirname "$0")/.." && pwd)"
defaults="$AP/Tools/autotest/default_params/copter.parm,$REPO/sim/vio-iris.parm"
"$AP/build/sitl/bin/arducopter" --model JSON --defaults "$defaults" -I0 -w \
    --home -35.363262,149.165237,584,0 > sitl.log 2>&1 &
pids+=($!)

echo "Gazebo and SITL are running. SITL waits for sim/fly_record.py (sitl.log: \"Waiting for connection\"). Ctrl-C to stop."
wait -n
echo "one of them stopped, see $WORLD/gazebo.log and $WORLD/sitl/sitl.log"
