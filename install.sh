#!/usr/bin/env bash
# Sets up the complete qb_arm environment on Ubuntu 24.04:
# ROS 2 Jazzy, MoveIt 2, Gazebo, Azure Kinect SDK, the qb_arm workspace and system config.
# Safe to re-run: every step checks what is already there.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

WS="$HOME/prj/ros2_ws"
GIT_BASE="git@github.com:whoobee"
BRANCH="develop"
REPOS=(qb_arm qb_arm_lite6 qb_arm_kinectdk_ros2 qb_arm_vision)
DO_UPGRADE=1
DO_KINECT=1
DO_BUILD=1
DO_BASHRC=1
DO_DISCOVERY=1
DO_REALTIME=1
DO_ESP=1
DO_MICROROS=1
DO_DOCS=1
DO_CONTROL=1
DO_MCP=1
DO_HANDS=1
DO_CLAW_AP=1
GRIPPER_DIR="$HOME/prj/qb_arm_gripper"
ACCEPT_K4A_EULA=0
DDS_IFACE=""                 # --dds-iface; default: the interface of the default route

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
  --no-docs             don't install the documentation server (qb-arm-docs.service, port 8080)
  --no-control          don't install the control center (qb-arm-control.service, port 8081)
  --no-mcp              don't install the MCP server for AI agents (qb-arm-mcp.service, port 8082)
  --no-hands            skip the hand tracker's Python environment (MediaPipe, ~/prj/venvs/hands)
  --no-claw-ap          don't set up qbarm-claw, the access point for the claw (needs a USB Wi-Fi adapter)
  --dds-iface IFACE     the one network interface ROS (Fast DDS) uses besides loopback / shared memory
                        (default: the interface of the default route; on qBArm the LAN cable enx00e04c360283)
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
        --no-docs) DO_DOCS=0 ;;
        --no-control) DO_CONTROL=0 ;;
        --no-mcp) DO_MCP=0 ;;
        --no-hands) DO_HANDS=0 ;;
        --no-claw-ap) DO_CLAW_AP=0 ;;
        --dds-iface) DDS_IFACE="$2"; shift ;;
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
    ros-jazzy-trac-ik-kinematics-plugin \
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
# One interface for ROS: two on the same subnet (Wi-Fi + LAN) made service replies get lost (spawner -> unconfigured
# trajectory controller, 2026-10-02)
[ -n "$DDS_IFACE" ] || DDS_IFACE=$(ip -o route show default | awk '{m = 0; d = ""; for (i = 1; i < NF; i++) {
    if ($i == "dev") d = $(i + 1); if ($i == "metric") m = $(i + 1) } print m, d}' | sort -n | head -1 | cut -d' ' -f2)
sed "s#@IFACE@#${DDS_IFACE:-lo}#" "$HERE/config/fastdds_qbarm.xml.in" > "$WS/fastdds_qbarm.xml"
info "wrote $WS/fastdds_qbarm.xml (ROS over ${DDS_IFACE:-lo only})"
# USB-C hub/Ethernet combo of the wired LAN: no USB autosuspend (it dropped out right after plugging in)
sudo install -m 644 "$HERE/config/90-qbarm-usb-eth.rules" /etc/udev/rules.d/90-qbarm-usb-eth.rules
sudo udevadm control --reload
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
if [ $DO_CLAW_AP -eq 1 ]; then
    # The claw's ESP32 sits on the arm among metal: through the building Wi-Fi it lost up to 75 % of its packets.
    # A second (USB) Wi-Fi adapter on qBArm runs a dedicated 2.4 GHz access point next to the arm instead.
    step "Claw access point qbarm-claw (second Wi-Fi adapter, 10.42.0.1/24)"
    apt_install iw
    AP_IF=""
    for dev in /sys/class/net/wlx*; do
        [ -e "$dev" ] || continue
        dev=$(basename "$dev")
        phy=$(iw dev "$dev" info 2>/dev/null | awk '/wiphy/{print "phy"$2}')
        if [ -n "$phy" ] && iw phy "$phy" info | sed -n '/Supported interface modes/,/Band/p' | grep -q '\* AP$'; then
            AP_IF=$dev; break
        fi
    done
    if [ -z "$AP_IF" ]; then
        info "no USB Wi-Fi adapter with access point support found - plug one in (e.g. TP-Link Archer T4U v3) and re-run"
    else
        sudo install -m 644 "$HERE/config/qbarm-claw-dnsmasq.conf" /etc/NetworkManager/dnsmasq-shared.d/qbarm-claw.conf
        # The password lives in the gripper's (git-ignored) wifi.env; create one if there is none yet
        ENV_FILE="$GRIPPER_DIR/wifi.env"
        PSK=$(grep -s '^QBAG_WIFI_PASSWORD=' "$ENV_FILE" | cut -d= -f2- || true)
        if [ -z "$PSK" ]; then
            PSK=$(python3 -c "import secrets,string; a=string.ascii_letters+string.digits; print(''.join(secrets.choice(a) for _ in range(20)))")
            if [ -f "$ENV_FILE" ]; then
                sed -i "s|^QBAG_WIFI_PASSWORD=.*|QBAG_WIFI_PASSWORD=$PSK|" "$ENV_FILE"
                info "generated the access point password and wrote it to $ENV_FILE"
            else
                info "no $ENV_FILE - access point password is in the NetworkManager connection (nmcli -s con show qbarm-claw)"
            fi
        fi
        nmcli -t -f NAME con show | grep -qx qbarm-claw || sudo nmcli con add type wifi con-name qbarm-claw ssid qbarm-claw >/dev/null
        sudo nmcli con modify qbarm-claw ifname "$AP_IF" autoconnect yes ssid qbarm-claw \
            802-11-wireless.mode ap 802-11-wireless.band bg 802-11-wireless.channel 1 \
            wifi-sec.key-mgmt wpa-psk wifi-sec.proto rsn wifi-sec.pairwise ccmp wifi-sec.group ccmp wifi-sec.psk "$PSK" \
            ipv4.method shared ipv4.addresses 10.42.0.1/24 ipv6.method disabled
        sudo nmcli con up qbarm-claw >/dev/null
        info "qbarm-claw on $AP_IF, channel 1 (pick a free one: sudo iw dev $AP_IF scan), claw at 10.42.0.10"
    fi
fi

# ---------------------------------------------------------------------------
if [ $DO_DOCS -eq 1 ]; then
    # Served from this checkout of qb_arm_install: run install.sh from a permanent clone (e.g. ~/prj/qb_arm_install)
    step "Documentation server (systemd: qb-arm-docs.service, http://<this machine>:8080)"
    sed "s#@USER@#$USER#; s#@DOCS@#$HERE/docs#g" "$HERE/config/qb-arm-docs.service.in" \
        | sudo tee /etc/systemd/system/qb-arm-docs.service >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable qb-arm-docs.service
    sudo systemctl restart qb-arm-docs.service
    info "status: $(systemctl is-active qb-arm-docs.service), pages from $HERE/docs/pages"
fi

# ---------------------------------------------------------------------------
if [ $DO_CONTROL -eq 1 ]; then
    # Needs the workspace built (qb_arm's control_center). It starts the cell, whose RViz opens on the desktop session.
    step "Control center (systemd: qb-arm-control.service, http://<this machine>:8081)"
    sed "s#@USER@#$USER#g; s#@UID@#$(id -u)#g; s#@HOME@#$HOME#g" "$HERE/config/qb-arm-control.service.in" \
        | sudo tee /etc/systemd/system/qb-arm-control.service >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable qb-arm-control.service
    sudo systemctl restart qb-arm-control.service
    info "status: $(systemctl is-active qb-arm-control.service)"
fi

if [ "$DO_MCP" = 1 ]; then
    # The arm as MCP tools for an AI agent (Hermes Agent on hbh-ai, voice through Home Assistant): its own venv with
    # the mcp package (2.0, as Hermes), a bearer token, a systemd service; it only talks to the control center
    step "MCP server (systemd: qb-arm-mcp.service, http://<this machine>:8082/mcp)"
    MCPV="$HOME/prj/venvs/mcp"
    [ -x "$MCPV/bin/python" ] || python3 -m venv "$MCPV"
    "$MCPV/bin/pip" install -q "mcp==2.0.0"
    mkdir -p "$HOME/.config/qbarm" && chmod 700 "$HOME/.config/qbarm"
    if [ ! -s "$HOME/.config/qbarm/mcp_token" ]; then
        python3 -c 'import secrets; print(secrets.token_urlsafe(32))' > "$HOME/.config/qbarm/mcp_token"
        chmod 600 "$HOME/.config/qbarm/mcp_token"
        info "new token in ~/.config/qbarm/mcp_token: give it to the agent (Authorization: Bearer <token>)"
    fi
    sed "s#@USER@#$USER#g; s#@HOME@#$HOME#g" "$HERE/config/qb-arm-mcp.service.in" \
        | sudo tee /etc/systemd/system/qb-arm-mcp.service >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable qb-arm-mcp.service
    sudo systemctl restart qb-arm-mcp.service
    info "status: $(systemctl is-active qb-arm-mcp.service)"
fi

if [ "$DO_HANDS" = 1 ]; then
    # qb_arm_vision hand_tracker: MediaPipe in its own venv (system site packages for ROS); mediapipe's own opencv
    # needs numpy 2, which the system's numpy-1 builds (scipy, matplotlib, cv_bridge) can't load: numpy < 2 and the
    # system OpenCV instead. The node re-executes itself in this venv.
    step "Hand tracker environment (MediaPipe, ~/prj/venvs/hands)"
    HANDS="$HOME/prj/venvs/hands"
    python3 -m venv --system-site-packages "$HANDS"
    "$HANDS/bin/pip" install -q mediapipe 'numpy<2'
    "$HANDS/bin/pip" uninstall -q -y opencv-contrib-python opencv-python 2>/dev/null || true
    mkdir -p "$HANDS/models"
    [ -f "$HANDS/models/hand_landmarker.task" ] || curl -fsSL -o "$HANDS/models/hand_landmarker.task" \
        https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task
    "$HANDS/bin/python" -c "import mediapipe, numpy; print('mediapipe', mediapipe.__version__, 'numpy', numpy.__version__)"
fi

# ---------------------------------------------------------------------------
cat <<EOF

==========================================
 qb_arm environment ready
==========================================
Open a new terminal (or: source $WS/ros_env.sh), then:

  cell start real       # whole cell: real Lite6 + claw + MoveIt + RViz + Kinect + qb_arm_vision
  cell start sim        # same with a simulated arm and claw
  cell stop             # always stop through cell: takes every node with it
  qbarm                 # bringup only (no vision); prefer cell
  qbarm camera:=false   # arm only
  kinect                # Kinect only

Other machines on the network join with:
  export ROS_DISCOVERY_SERVER=<this machine's IP>:11811 ROS_SUPER_CLIENT=TRUE

Documentation: http://<this machine's IP>:8080

Log out and back in once so the 'realtime' and 'dialout' groups apply.
EOF
