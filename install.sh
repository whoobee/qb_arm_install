#!/bin/bash

# ROS2 Workspace Installation Script
# This script sets up ROS2 Jazzy and all dependencies for the QB Arm workspace

set -e  # Exit on error

echo "=========================================="
echo "ROS2 Jazzy + QB Arm Workspace Setup"
echo "=========================================="

# Check if running on Ubuntu
if ! grep -qi ubuntu /etc/os-release; then
    echo "Error: This script is designed for Ubuntu. Please install Ubuntu first."
    exit 1
fi

# Get Ubuntu version
UBUNTU_VERSION=$(lsb_release -cs)
echo "Detected Ubuntu version: $UBUNTU_VERSION"

# Step 1: Update system packages
echo ""
echo "Step 1: Updating system packages..."
sudo apt update
sudo apt upgrade -y

# Step 2: Install ROS2 Jazzy
echo ""
echo "Step 2: Installing ROS2 Jazzy..."

# Add ROS2 GPG key
sudo curl -sSL https://repo.ros2.org/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://repo.ros2.org/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package list
sudo apt update

# Install ROS2 Jazzy desktop
sudo apt install -y ros-jazzy-desktop

# Install development tools
echo ""
echo "Step 3: Installing ROS2 development tools..."
sudo apt install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-control-toolbox \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential

# Step 4: Install MoveIt2
echo ""
echo "Step 4: Installing MoveIt2..."
sudo apt install -y \
    ros-jazzy-moveit \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-task-constructor \
    ros-jazzy-moveit-configs-utils

# Step 5: Install Azure Kinect dependencies
echo ""
echo "Step 5: Installing Azure Kinect dependencies..."
sudo apt install -y \
    libk4a1.3 \
    libk4a1.3-dev \
    k4a-tools \
    libsoundio-dev \
    pkg-config \
    curl

# Step 6: Install perception packages
echo ""
echo "Step 6: Installing perception packages..."
sudo apt install -y \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-sensor-msgs \
    ros-jazzy-pcl-ros \
    ros-jazzy-pointcloud-to-laserscan

# Step 7: Install visualization tools
echo ""
echo "Step 7: Installing visualization tools..."
sudo apt install -y \
    ros-jazzy-rviz2 \
    ros-jazzy-rviz-imu-plugin \
    ros-jazzy-rviz-visual-tools

# Step 8: Install other dependencies
echo ""
echo "Step 8: Installing additional dependencies..."
sudo apt install -y \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-geometry2 \
    ros-jazzy-angles \
    ros-jazzy-diagnostic-aggregator \
    python3-pip \
    git \
    cmake

# Step 9: Initialize rosdep
echo ""
echo "Step 9: Initializing rosdep..."
sudo rosdep init || true
rosdep update

# Step 10: Clone required repositories
echo ""
echo "Step 10: Cloning QB Arm repositories..."
cd ~/ros2_ws/src

# Clone QB Arm repository
if [ ! -d "qb_arm" ]; then
    echo "Cloning qb_arm..."
    git clone https://github.com/whoobee/qb_arm.git
else
    echo "qb_arm already exists, skipping clone..."
fi

# Clone QB Arm Kinect DK ROS2 repository
if [ ! -d "qb_arm_kinectdk_ros2" ]; then
    echo "Cloning qb_arm_kinectdk_ros2..."
    git clone https://github.com/whoobee/qb_arm_kinectdk_ros2.git
else
    echo "qb_arm_kinectdk_ros2 already exists, skipping clone..."
fi

# Step 11: Install workspace dependencies
echo ""
echo "Step 10: Installing workspace dependencies..."
cd ~/ros2_ws

# Install dependencies from source packages
if [ -f "src/package.rosinstall" ]; then
    echo "Installing from rosinstall file..."
    rosdep install --from-paths src --ignore-src -r -y
fi

# Install rosdeps for all packages
rosdep install --from-paths src --ignore-src -r -y || true

# Step 11: Build Python packages
echo ""
echo "Step 11: Installing Python dependencies..."
pip3 install -q \
    numpy \
    opencv-python \
    Pillow \
    pyyaml

# Step 12: Build the workspace
echo ""
echo "Step 12: Building ROS2 workspace..."
echo "This may take several minutes..."
colcon build --symlink-install --continue-on-error

# Step 13: Source the workspace
echo ""
echo "Step 13: Setting up workspace sourcing..."

# Add to ~/.bashrc if not already present
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Workspace" >> ~/.bashrc
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
fi

# Step 14: Verify installation
echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "To activate the ROS2 environment, run:"
echo "  source ~/.bashrc"
echo ""
echo "Or manually source:"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  source ~/ros2_ws/install/setup.bash"
echo ""
echo "To verify installation:"
echo "  ros2 --version"
echo "  ros2 run demo_nodes_cpp listener &"
echo "  ros2 run demo_nodes_cpp talker"
echo ""
echo "To launch the QB Arm system:"
echo "  ros2 launch qb_arm qb_arm_launch.py"
echo ""
echo "For more information, see README.md files in the workspace."
echo ""
