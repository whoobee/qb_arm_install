# QB Arm Installation Repository

Automated setup script for installing ROS2 Jazzy and all dependencies required to run the QB Arm robotic manipulation system. This repository provides a one-command installation process for setting up a complete ROS2 development environment with Azure Kinect integration and MoveIt2 motion planning.

## Overview

This repository contains a comprehensive bash installation script (`install.sh`) that automates the entire setup process for the QB Arm workspace, including:

- **ROS2 Jazzy Framework** - Latest ROS2 distribution with full desktop installation
- **Robot Control Stack** - ros2-control, ros2-controllers, and hardware interface packages
- **Motion Planning** - MoveIt2 with Servo capabilities
- **Depth Perception** - Azure Kinect SDK and drivers
- **Visualization** - RViz2 with plugins and visualization tools
- **Project Repositories** - Automatic cloning of QB Arm packages
- **Development Tools** - CMake, git, Python tools, and more

## System Requirements

### Operating System
- **Ubuntu 22.04 LTS** (Jammy) - Recommended
- **Ubuntu 24.04 LTS** (Noble) - Supported
- Linux kernel 5.15 or later

### Hardware
- **Processor**: Intel i5/i7 or equivalent (multi-core recommended)
- **RAM**: Minimum 8GB (16GB+ recommended for simulation)
- **Storage**: 20GB free disk space minimum
- **USB**: USB 3.0 ports for Azure Kinect sensor

### Network
- Stable internet connection (script downloads ~2-3GB of packages)
- Static IP or DHCP for robot arm (192.168.1.23 by default)

## Prerequisites

Before running the installation script, ensure you have:

1. **Ubuntu installed** on your machine
2. **sudo access** (script requires administrative privileges)
3. **Internet connection** (required for package downloads)
4. **~20GB free disk space**

## Installation

### Quick Start

```bash
# Navigate to the qb_arm_install directory
cd ~/ros2_ws/src/qb_arm_install

# Make the script executable (if not already)
chmod +x install.sh

# Run the installation
./install.sh
```

### Step-by-Step

The installation script performs 14 automated steps:

1. **System Update** - Updates Ubuntu package lists and upgrades system
2. **ROS2 Jazzy Installation** - Installs ROS2 desktop with development tools
3. **Development Tools** - Installs build tools, cmake, git, and ROS2 utilities
4. **MoveIt2 Installation** - Installs motion planning framework and Servo
5. **Azure Kinect SDK** - Installs Kinect drivers and development libraries
6. **Perception Packages** - Installs CV bridge, image transport, PCL, point clouds
7. **Visualization Tools** - Installs RViz2 and visualization plugins
8. **Additional Dependencies** - Installs TF2, geometry packages, and diagnostic tools
9. **ROS Dependency Manager** - Initializes rosdep for dependency management
10. **Repository Cloning** - Clones required QB Arm repositories
11. **Workspace Dependencies** - Installs all ROS package dependencies
12. **Python Dependencies** - Installs NumPy, OpenCV, Pillow, YAML
13. **Workspace Build** - Builds all ROS2 packages using colcon
14. **Environment Setup** - Configures shell environment for ROS2

### Installation Time

Expected installation duration: **15-30 minutes**
- System updates: 2-5 minutes
- ROS2 and packages: 10-15 minutes
- Repository cloning: 1-2 minutes
- Workspace build: 5-10 minutes

## Packages Installed

### ROS2 Core
- `ros-jazzy-desktop` - Complete ROS2 distribution
- `ros-jazzy-ros2-control` - Hardware control interface
- `ros-jazzy-ros2-controllers` - Control plugins

### Motion Planning & Manipulation
- `ros-jazzy-moveit` - Motion planning framework
- `ros-jazzy-moveit-servo` - Real-time servo control
- `ros-jazzy-moveit-task-constructor` - Task-level planning

### Perception & Vision
- `ros-jazzy-cv-bridge` - OpenCV-ROS bridge
- `ros-jazzy-pcl-ros` - Point Cloud Library integration
- `ros-jazzy-image-transport` - Efficient image communication

### Visualization
- `ros-jazzy-rviz2` - 3D visualization tool
- `ros-jazzy-rviz-visual-tools` - Visualization plugins

### System Tools
- `python3-rosdep` - ROS dependency manager
- `python3-pip` - Python package manager
- `cmake` - Build system
- `git` - Version control

## Cloned Repositories

The script automatically clones the following GitHub repositories into `~/ros2_ws/src/`:

1. **qb_arm** - Main QB Arm ROS2 package
   - Static transform broadcaster for camera calibration
   - Launch configurations and URDF files
   - RViz setup files

2. **qb_arm_kinectdk_ros2** - Azure Kinect integration package
   - Kinect-specific drivers and configurations
   - Depth image processing
   - Point cloud generation

## Configuration

### Default Robot Parameters

The script uses the following default parameters for the QB Arm setup:

| Parameter | Default Value | Purpose |
|-----------|---------------|---------|
| Robot IP | 192.168.1.23 | Network address of Lite6 arm |
| DOF | 6 | Degrees of freedom |
| Robot Type | lite | uFactory Lite6 |
| Color Resolution | 720P | Azure Kinect resolution |
| Depth Mode | WFOV_2X2BINNED | Kinect depth mode |
| FPS | 30 | Frames per second |

To change these parameters, edit the launch file after installation:
```bash
nano ~/ros2_ws/src/qb_arm/launch/qb_arm_launch.py
```

### Environment Setup

The script automatically adds the following to `~/.bashrc`:

```bash
# ROS2 Workspace
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

Apply the changes immediately:
```bash
source ~/.bashrc
```

## Post-Installation Verification

### Test ROS2 Installation

```bash
# Check ROS2 version
ros2 --version

# List installed packages
ros2 pkg list | head -20

# Test basic communication
ros2 run demo_nodes_cpp listener &
ros2 run demo_nodes_cpp talker
```

### Test QB Arm Setup

```bash
# Launch the complete QB Arm system
ros2 launch qb_arm qb_arm_launch.py

# In another terminal, check available nodes
ros2 node list

# Check active topics
ros2 topic list

# View camera data
ros2 topic echo /depth/image_raw
```

### Verify Hardware Connection

```bash
# Check if Azure Kinect is detected
k4a-viewer

# Verify robot connectivity
ping 192.168.1.23
```

## Troubleshooting

### Installation Failures

**Issue: "Permission denied" error**
```bash
# Solution: Make script executable
chmod +x install.sh

# Or run with sudo
sudo bash install.sh
```

**Issue: "Ubuntu not detected" error**
```bash
# Solution: Script is designed for Ubuntu only
# Verify your OS:
cat /etc/os-release
```

**Issue: Network-related failures during downloads**
```bash
# Solution: Check internet connection and retry
ping google.com

# Try running the script again (it will skip completed steps)
./install.sh
```

### ROS2 Initialization Issues

**Issue: "rosdep: command not found"**
```bash
# Solution: Reinstall rosdep
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

**Issue: Source command not working**
```bash
# Solution: Manually source environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Hardware Connection Issues

**Issue: Azure Kinect not detected**
```bash
# Check USB connection
lsusb | grep Kinect

# Install additional dependencies
sudo apt install -y libsoundio-dev
```

**Issue: Robot cannot be reached**
```bash
# Check network configuration
ip addr show
ping 192.168.1.23

# Update robot IP in launch configuration
```

## Manual Installation (Alternative)

If you prefer to install components manually or need customization:

```bash
# 1. Install ROS2 Jazzy
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop

# 2. Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 3. Clone repositories
git clone https://github.com/whoobee/qb_arm.git
git clone https://github.com/whoobee/qb_arm_kinectdk_ros2.git

# 4. Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 5. Build workspace
colcon build --symlink-install
```

## Updates and Maintenance

### Updating Packages

```bash
# Update ROS2 packages
sudo apt update
sudo apt upgrade -y

# Update workspace packages
cd ~/ros2_ws
git -C src/qb_arm pull
git -C src/qb_arm_kinectdk_ros2 pull
colcon build --symlink-install
```

### Cleaning Build Artifacts

```bash
# Clean build, install, and log directories
cd ~/ros2_ws
rm -rf build/ install/ log/

# Rebuild workspace
colcon build --symlink-install
```

## Documentation

For detailed information about the QB Arm package and its components, see:

- [QB Arm README](../qb_arm/README.md) - Main package documentation
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)
- [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK)

## Support

For issues or questions:

1. Check the [Troubleshooting](#troubleshooting) section above
2. Review ROS2 and MoveIt2 documentation
3. Check Azure Kinect SDK documentation
4. Open an issue on the [QB Arm GitHub repository](https://github.com/whoobee/qb_arm)

## License

This installation script and documentation are provided as-is. Refer to individual package licenses for terms.

## Repository Information

- **Repository**: qb_arm_install
- **Maintainer**: whoobee
- **Last Updated**: January 2026
- **ROS2 Version**: Jazzy
- **Ubuntu Versions**: 22.04 LTS, 24.04 LTS

## Contributing

To improve this installation script:

1. Test on a fresh Ubuntu installation
2. Document any issues or improvements
3. Submit pull requests with clear descriptions
4. Ensure the script remains idempotent (safe to run multiple times)

---

**Happy robotics! 🤖**
