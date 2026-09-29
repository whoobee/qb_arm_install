#!/usr/bin/env bash
# Sets up the complete qb_arm environment on Ubuntu 24.04:
# ROS 2 Jazzy, MoveIt 2, Gazebo, Azure Kinect SDK, the qb_arm workspace and system config.
# Safe to re-run: every step checks what is already there.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WS="$HOME/prj/ros2_ws"
GIT_BASE="git@github.com:whoobee"
BRANCH="develop"
REPOS=(qb_arm qb_arm_lite6 qb_arm_kinectdk_ros2)
DO_UPGRADE=1
DO_KINECT=1
DO_BUILD=1
DO_BASHRC=1
DO_DISCOVERY=1
DO_REALTIME=1
DO_ESP=1
DO_MICROROS=1
GRIPPER_DIR="$HOME/prj/qb_arm_gripper"
ACCEPT_K4A_EULA=0

K4A_URL="https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk"
K4A_DEBS=(
    "libk4a1.4/libk4a1.4_1.4.1_amd64.deb c1c63f81641eed1326136a44e5a5fd229e1e6315b2b20b2dddb5523a556c9329"
    "libk4a1.4-dev/libk4a1.4-dev_1.4.1_amd64.deb 08303094b9ad36ea74c19bc8b8950c97055e73dd2e8bd18e2af5e165a2289cd2"
)

usage() {
    cat <<EOF
Usage: ./install.sh [options]

  --ws DIR              workspace directory (default: $WS)
  --https               clone over HTTPS instead of SSH (repos must be readable)
  --branch NAME         branch to check out in the qb_arm repos (default: $BRANCH)
  --accept-k4a-eula     accept Microsoft's Azure Kinect SDK EULA without prompting
  --no-upgrade          skip 'apt upgrade'
  --no-kinect           skip the Azure Kinect SDK and udev rule
  --no-build            skip rosdep install + colcon build of the workspace
  --no-bashrc           don't add the ROS environment to ~/.bashrc
  --no-discovery-server don't install the Fast DDS discovery server service
  --no-realtime         don't grant real-time scheduling to this user
  --no-esp              skip the ESP32 tools (dialout, esptool, PlatformIO) and the qb_arm_gripper clone
  --no-microros-agent   skip the micro-ROS agent (build + ros2-microros-agent.service) for the gripper
  -h, --help            show this help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --ws) WS="$(realpath -m "$2")"; shift ;;
        --https) GIT_BASE="https://github.com/whoobee" ;;
        --branch) BRANCH="$2"; shift ;;
        --accept-k4a-eula) ACCEPT_K4A_EULA=1 ;;
        --no-upgrade) DO_UPGRADE=0 ;;
        --no-kinect) DO_KINECT=0 ;;
        --no-build) DO_BUILD=0 ;;
        --no-bashrc) DO_BASHRC=0 ;;
        --no-discovery-server) DO_DISCOVERY=0 ;;
        --no-realtime) DO_REALTIME=0 ;;
        --no-esp) DO_ESP=0 ;;
        --no-microros-agent) DO_MICROROS=0 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
    shift
done

STEP=0
step() { STEP=$((STEP + 1)); echo; echo "=== [$STEP] $* ==="; }
info() { echo "  - $*"; }
die() { echo "ERROR: $*" >&2; exit 1; }

# Retry flaky network commands: retry <cmd...>
retry() {
    local n
    for n in 1 2 3 4; do
        "$@" && return 0
        [ $n -lt 4 ] && { info "failed, retrying in $((n * 5)) s..." >&2; sleep $((n * 5)); }
    done
    return 1
}

# ROS setup scripts reference unset variables
source_ros() { set +u; source /opt/ros/jazzy/setup.bash; [ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"; set -u; }

# Clone into a fresh directory (a failed attempt must not leave a partial clone behind)
clone() { rm -rf "$3"; git clone -q -b "$1" "$2" "$3"; }

apt_install() { sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"; }

# ---------------------------------------------------------------------------
step "Checking system"
[ "$EUID" -ne 0 ] || die "run as your normal user (not root/sudo); the script calls sudo itself"
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "noble" ] || die "Ubuntu 24.04 (noble) required for ROS 2 Jazzy, found: ${PRETTY_NAME:-unknown}"
[ "$(dpkg --print-architecture)" = "amd64" ] || die "amd64 required (the Azure Kinect SDK is amd64-only)"
info "$PRETTY_NAME, workspace: $WS"
sudo true  # ask for the password once, up front

# ---------------------------------------------------------------------------
step "System packages"
sudo apt-get update
[ $DO_UPGRADE -eq 1 ] && sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
apt_install software-properties-common curl git rsync locales ca-certificates
sudo add-apt-repository -y universe
if ! locale | grep -qi 'utf-8'; then
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
fi

# ---------------------------------------------------------------------------
step "ROS 2 apt repository"
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
    info "ros2-apt-source already installed"
else
    ver=$(retry curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}')
    [ -n "$ver" ] || die "could not determine the latest ros-apt-source release"
    tmp=$(mktemp -d)
    retry curl -fsSL -o "$tmp/ros2-apt-source.deb" \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${VERSION_CODENAME}_all.deb"
    sudo apt-get install -y "$tmp/ros2-apt-source.deb"
    rm -rf "$tmp"
fi
sudo apt-get update

# ---------------------------------------------------------------------------
step "ROS 2 Jazzy, MoveIt 2, Gazebo and tools"
apt_install \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-moveit \
    ros-jazzy-ros-gz \
    python3-numpy \
    python3-scipy \
    python3-yaml

# ---------------------------------------------------------------------------
if [ $DO_KINECT -eq 1 ]; then
    step "Azure Kinect Sensor SDK 1.4.1"
    if dpkg -s libk4a1.4-dev >/dev/null 2>&1; then
        info "libk4a1.4-dev already installed"
    else
        tmp=$(mktemp -d)
        for entry in "${K4A_DEBS[@]}"; do
            read -r path sha <<<"$entry"
            info "downloading $(basename "$path")"
            retry curl -fsSL -o "$tmp/$(basename "$path")" "$K4A_URL/$path"
            echo "$sha  $tmp/$(basename "$path")" | sha256sum -c --quiet || die "checksum mismatch for $path"
        done
        # Microsoft only ships these for 18.04, but they work on 24.04.
        # libk4a1.4 asks to accept the EULA (/usr/share/doc/libk4a1.4/LICENSE.txt after install).
        if [ $ACCEPT_K4A_EULA -eq 1 ]; then
            sudo ACCEPT_EULA=Y apt-get install -y "$tmp"/*.deb
        else
            info "libk4a1.4 will ask you to accept Microsoft's EULA"
            sudo apt-get install -y "$tmp"/*.deb
        fi
        rm -rf "$tmp"
    fi
    info "udev rule (camera access without root)"
    sudo install -m 644 "$HERE/config/99-k4a.rules" /etc/udev/rules.d/99-k4a.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger --action=add --attr-match=idVendor=045e || true
fi

# ---------------------------------------------------------------------------
step "rosdep"
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
retry rosdep update

# ---------------------------------------------------------------------------
step "Workspace and repositories"
mkdir -p "$WS/src"
for repo in "${REPOS[@]}"; do
    dir="$WS/src/$repo"
    if [ -d "$dir/.git" ]; then
        info "$repo already cloned ($(git -C "$dir" rev-parse --abbrev-ref HEAD) @ $(git -C "$dir" rev-parse --short HEAD)), leaving it as is"
    elif [ -e "$dir" ]; then
        die "$dir exists but is not a git clone - move it away and re-run"
    else
        info "cloning $repo ($BRANCH)"
        retry clone "$BRANCH" "$GIT_BASE/$repo.git" "$dir"
    fi
done
if [ $DO_MICROROS -eq 1 ]; then
    # micro-ROS agent (not packaged for Jazzy): the claw's ESP32 talks to ROS through it
    for pair in "micro-ROS-Agent micro_ros_agent_repo" "micro_ros_msgs micro_ros_msgs"; do
        set -- $pair
        if [ -d "$WS/src/$2/.git" ]; then
            info "$2 already cloned, leaving it as is"
        else
            info "cloning $1 (jazzy)"
            retry clone jazzy "https://github.com/micro-ROS/$1.git" "$WS/src/$2"
        fi
    done
fi

# ---------------------------------------------------------------------------
if [ $DO_BUILD -eq 1 ]; then
    step "Workspace dependencies (rosdep)"
    source_ros
    # K4A = the Azure Kinect SDK, installed above from Microsoft's .debs
    rosdep install --from-paths "$WS/src" --ignore-src --rosdistro jazzy -y --skip-keys K4A

    step "Building the workspace (takes a few minutes)"
    (cd "$WS" && colcon build --symlink-install)
fi

# ---------------------------------------------------------------------------
step "ROS environment"
sed "s#@WS@#$WS#" "$HERE/config/ros_env.sh.in" > "$WS/ros_env.sh"
info "wrote $WS/ros_env.sh"
if [ $DO_BASHRC -eq 1 ]; then
    line="source $WS/ros_env.sh"
    if grep -qxF "$line" ~/.bashrc; then
        info "$HOME/.bashrc already sources it"
    else
        printf '\n# ROS 2 / qb_arm workspace\n%s\n' "$line" >> ~/.bashrc
        info "added to ~/.bashrc"
    fi
    if grep -qE '^source (/opt/ros/jazzy/setup\.bash|~/ros2_ws/install/setup\.bash)$' ~/.bashrc; then
        info "NOTE: ~/.bashrc also has older ROS 'source' lines - remove them, ros_env.sh covers them"
    fi
fi

# ---------------------------------------------------------------------------
if [ $DO_DISCOVERY -eq 1 ]; then
    step "Fast DDS discovery server (systemd: ros2-discovery.service, UDP 11811)"
    sed "s#@USER@#$USER#" "$HERE/config/ros2-discovery.service.in" \
        | sudo tee /etc/systemd/system/ros2-discovery.service >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable ros2-discovery.service
    sudo systemctl restart ros2-discovery.service
    info "status: $(systemctl is-active ros2-discovery.service)"
fi

# ---------------------------------------------------------------------------
if [ $DO_MICROROS -eq 1 ] && [ $DO_BUILD -eq 1 ]; then
    step "micro-ROS agent (systemd: ros2-microros-agent.service, UDP 8888)"
    sed "s#@USER@#$USER#; s#@WS@#$WS#" "$HERE/config/ros2-microros-agent.service.in" \
        | sudo tee /etc/systemd/system/ros2-microros-agent.service >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable ros2-microros-agent.service
    sudo systemctl restart ros2-microros-agent.service
    info "status: $(systemctl is-active ros2-microros-agent.service)"
fi

# ---------------------------------------------------------------------------
if [ $DO_REALTIME -eq 1 ]; then
    step "Real-time scheduling for ros2_control"
    sudo groupadd -f realtime
    sudo usermod -aG realtime "$USER"
    sudo install -m 644 "$HERE/config/99-realtime.conf" /etc/security/limits.d/99-realtime.conf
    info "$USER is in group 'realtime' (takes effect after the next login)"
fi

# ---------------------------------------------------------------------------
if [ $DO_ESP -eq 1 ]; then
    step "ESP32 tools (serial access, esptool, PlatformIO Core)"
    apt_install python3-pip python3-venv pipx python3-serial
    sudo usermod -aG dialout "$USER"
    info "$USER is in group 'dialout' (takes effect after the next login)"
    if pipx list --short 2>/dev/null | grep -q '^esptool '; then
        pipx upgrade esptool
    else
        pipx install esptool
    fi
    retry curl -fsSL -o /tmp/get-platformio.py \
        https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 /tmp/get-platformio.py
    rm -f /tmp/get-platformio.py
    pipx ensurepath >/dev/null
    mkdir -p "$HOME/.local/bin"
    for b in pio platformio piodebuggdb; do
        ln -sf "$HOME/.platformio/penv/bin/$b" "$HOME/.local/bin/$b"
    done
    retry curl -fsSL -o /tmp/99-platformio-udev.rules \
        https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules
    sudo install -m 644 /tmp/99-platformio-udev.rules /etc/udev/rules.d/99-platformio-udev.rules
    rm -f /tmp/99-platformio-udev.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    info "esptool $("$HOME/.local/bin/esptool" version | tail -1), $("$HOME/.local/bin/pio" --version)"
    if [ -d "$GRIPPER_DIR/.git" ]; then
        info "qb_arm_gripper already cloned, leaving it as is"
    else
        info "cloning qb_arm_gripper ($BRANCH) to $GRIPPER_DIR"
        retry clone "$BRANCH" "$GIT_BASE/qb_arm_gripper.git" "$GRIPPER_DIR"
    fi
    [ -f "$GRIPPER_DIR/wifi.env" ] || cp "$GRIPPER_DIR/wifi.env.example" "$GRIPPER_DIR/wifi.env"
    info "gripper firmware: fill in $GRIPPER_DIR/wifi.env, then: cd $GRIPPER_DIR && pio run -e gripper -t upload"
fi

# ---------------------------------------------------------------------------
cat <<EOF

==========================================
 qb_arm environment ready
==========================================
Open a new terminal (or: source $WS/ros_env.sh), then:

  qbarm                 # real Lite6 (192.168.1.23) + MoveIt + RViz + Kinect
  qbarm sim:=true       # simulated arm + Kinect
  qbarm camera:=false   # arm only
  kinect                # Kinect only

Other machines on the network join with:
  export ROS_DISCOVERY_SERVER=<this machine's IP>:11811 ROS_SUPER_CLIENT=TRUE

Log out and back in once so the 'realtime' and 'dialout' groups apply.
EOF
