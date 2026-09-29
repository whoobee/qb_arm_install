# qb_arm_install

One script that sets up the complete qb_arm environment on a fresh **Ubuntu 24.04** machine:
ROS 2 Jazzy, MoveIt 2, the UFactory Lite6 driver, the ceiling Azure Kinect DK and the qb_arm bringup.

```bash
git clone git@github.com:whoobee/qb_arm_install.git
cd qb_arm_install
./install.sh
```

Run it as your normal user (it calls `sudo` itself). It is safe to re-run: every step checks what is
already there. Takes roughly 15–30 minutes, mostly downloads and the workspace build.

## What it does

| Step | Details |
|---|---|
| System packages | `apt upgrade`, `universe` repo, UTF-8 locale |
| ROS 2 Jazzy | official `ros2-apt-source` repo, `ros-jazzy-desktop`, `ros-dev-tools` (colcon, rosdep, vcs) |
| MoveIt 2 + Gazebo | `ros-jazzy-moveit`, `ros-jazzy-ros-gz` (Harmonic) |
| Azure Kinect SDK | Microsoft's `libk4a1.4` / `libk4a1.4-dev` 1.4.1 `.deb`s (checksum-verified) and the `99-k4a.rules` udev rule |
| Workspace | clones the repos below into `~/prj/ros2_ws/src`, `rosdep install`, `colcon build --symlink-install` |
| Environment | writes `~/prj/ros2_ws/ros_env.sh` and sources it from `~/.bashrc` |
| Network | Fast DDS discovery server as systemd service `ros2-discovery.service` (UDP 11811) |
| Real-time | `realtime` group + `/etc/security/limits.d/99-realtime.conf` for ros2_control |
| ESP32 | `dialout` group, pip/venv/pipx, `esptool` (pipx), PlatformIO Core (`pio`) + its udev rules, [qb_arm_gripper](https://github.com/whoobee/qb_arm_gripper) cloned to `~/prj/qb_arm_gripper` |
| micro-ROS agent | micro-ROS-Agent + micro_ros_msgs (jazzy) built in the workspace, systemd `ros2-microros-agent.service` on UDP 8888 for the claw's ESP32 |

Repositories (branch `develop`):

| Repo | Contents |
|---|---|
| [qb_arm](https://github.com/whoobee/qb_arm) | bringup launch files, camera pose + calibration tools |
| [qb_arm_lite6](https://github.com/whoobee/qb_arm_lite6) | UFactory `xarm_ros2` (jazzy) – Lite6 driver, MoveIt config |
| [qb_arm_kinectdk_ros2](https://github.com/whoobee/qb_arm_kinectdk_ros2) | Azure Kinect ROS driver, patched for Jazzy |

### Options

```
--ws DIR              workspace directory (default: ~/prj/ros2_ws)
--https               clone over HTTPS instead of SSH (repos must be readable)
--branch NAME         branch to check out in the qb_arm repos (default: develop)
--accept-k4a-eula     accept Microsoft's Azure Kinect SDK EULA without prompting
--no-upgrade          skip 'apt upgrade'
--no-kinect           skip the Azure Kinect SDK and udev rule
--no-build            skip rosdep install + colcon build of the workspace
--no-bashrc           don't add the ROS environment to ~/.bashrc
--no-discovery-server don't install the Fast DDS discovery server service
--no-realtime         don't grant real-time scheduling to this user
--no-esp              skip the ESP32 tools (dialout, esptool, PlatformIO) and the qb_arm_gripper clone
--no-microros-agent   skip the micro-ROS agent (build + ros2-microros-agent.service) for the gripper
```

The repos are private, so cloning over SSH needs a key on your GitHub account
(`ssh-keygen -t ed25519`, then add `~/.ssh/id_ed25519.pub` under GitHub → Settings → SSH keys).

The Azure Kinect SDK asks you to accept Microsoft's EULA during install (or pass `--accept-k4a-eula`).

## After installing

Log out and back in once (for the `realtime` and `dialout` groups), plug in the Kinect, then in a new terminal:

```bash
qbarm                 # real Lite6 (192.168.1.23) + MoveIt + RViz + Kinect
qbarm sim:=true       # simulated arm + Kinect
qbarm camera:=false   # arm only
kinect                # Kinect only
cb                    # rebuild the workspace and re-source it
```

`qbarm` / `kinect` are aliases for `ros2 launch qb_arm bringup.launch.py` / `kinect.launch.py`;
see the [qb_arm README](https://github.com/whoobee/qb_arm) for launch arguments and camera calibration.

Quick checks:

```bash
ros2 run demo_nodes_cpp talker & ros2 run demo_nodes_py listener   # ROS works
systemctl status ros2-discovery                                    # discovery server running
lsusb | grep -i "Azure Kinect"                                     # camera connected
ulimit -r                                                          # 99 after re-login
```

## Network (ROS "master")

ROS 2 has no master; nodes find each other through DDS discovery. To make this machine the central point
the way a ROS 1 master would be, it runs a Fast DDS **discovery server** on UDP port 11811, and
`ros_env.sh` points the local nodes at it. Other machines join with:

```bash
export ROS_DOMAIN_ID=0
export ROS_DISCOVERY_SERVER=<this machine's IP>:11811
export ROS_SUPER_CLIENT=TRUE   # only needed for ros2 topic/node list there
```

Give this machine a fixed IP (e.g. a DHCP reservation on the router) so that address doesn't change.

## Troubleshooting

- **Arm doesn't move:** check `ros2 topic echo --once /ufactory/robot_states` – a non-zero `err` is a
  controller fault (look it up in UFactory Studio at `http://<robot_ip>:18333`; power-cycling the arm
  clears many servo faults).
- **Kinect node aborts with `depth_mode ... does not support ... fps`:** the driver's default
  `WFOV_UNBINNED` only supports 15 fps; the qb_arm launch files use `NFOV_UNBINNED` at 30 fps.
- **Kinect not accessible without root:** re-plug the camera after the udev rule is installed.
- **Clone fails with `Permission denied (publickey)`:** add your SSH key to GitHub (see above).
