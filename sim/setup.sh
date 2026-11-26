#!/usr/bin/env bash
# Installs the simulation tools on Ubuntu 22.04 or 24.04 (native Linux, with a graphics card):
# Gazebo Harmonic, ArduPilot SITL from the Copter-4.6.3 tag (the same flight code as on our FC),
# the ardupilot_gazebo plugin, and what this repository needs to build.
#
# Everything outside apt goes into ~/sim (SIM_DIR to change it), nothing into this repository.
# Asks for your sudo password. ArduPilot's own prerequisite script also adds lines to ~/.profile
# and, on 24.04, makes the Python venv ~/venv-ardupilot.
#
# Run as your user, not with sudo:   sim/setup.sh

set -euo pipefail

SIM_DIR="${SIM_DIR:-$HOME/sim}"
PLUGIN_COMMIT=f116b03   # ardupilot_gazebo version these scripts were written against

[[ $EUID -ne 0 ]] || { echo "error: run as your user, not as root" >&2; exit 1; }
. /etc/os-release
[[ "$ID" == ubuntu && ( "$VERSION_ID" == 22.04 || "$VERSION_ID" == 24.04 ) ]] ||
    { echo "error: needs Ubuntu 22.04 or 24.04, this is $PRETTY_NAME" >&2; exit 1; }

mkdir -p "$SIM_DIR"

echo "== Gazebo Harmonic (packages.osrfoundation.org)"
sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg git
sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y gz-harmonic libgz-sim8-dev rapidjson-dev \
    python3-gz-transport13 python3-gz-msgs10 python3-opencv python3-numpy python3-yaml \
    libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    cmake ninja-build g++ libeigen3-dev libyaml-cpp-dev

echo "== ArduPilot Copter-4.6.3"
if [[ ! -d "$SIM_DIR/ardupilot" ]]; then
    git clone --branch Copter-4.6.3 --depth 1 --recurse-submodules --shallow-submodules \
        https://github.com/ArduPilot/ardupilot.git "$SIM_DIR/ardupilot"
fi
(
    cd "$SIM_DIR/ardupilot"
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    set +u
    [[ -f "$HOME/venv-ardupilot/bin/activate" ]] && . "$HOME/venv-ardupilot/bin/activate"
    set -u
    ./waf configure --board sitl
    ./waf copter
)

echo "== ardupilot_gazebo plugin"
if [[ ! -d "$SIM_DIR/ardupilot_gazebo" ]]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$SIM_DIR/ardupilot_gazebo"
fi
git -C "$SIM_DIR/ardupilot_gazebo" checkout -q "$PLUGIN_COMMIT"
GZ_VERSION=harmonic cmake -S "$SIM_DIR/ardupilot_gazebo" -B "$SIM_DIR/ardupilot_gazebo/build" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$SIM_DIR/ardupilot_gazebo/build" -j 4

echo
echo "Done. Log out and in again (or: source ~/.profile), then follow sim/README.md."
