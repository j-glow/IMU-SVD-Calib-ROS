# Full Setup Guide

This guide will walk you through the complete setup process for the IMU-SVD-Calib-ROS package.

## 1. Install ROS 2
Ensure you have a working ROS 2 environment installed on your system (e.g., Humble, Iron, or Rolling).
Follow the official ROS 2 installation instructions for your operating system:
[ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html)

## 2. Install Dependencies
You will need a C++17 compatible compiler and Eigen3.
On Ubuntu, you can install the necessary dependencies via:
```bash
sudo apt update
sudo apt install build-essential cmake
sudo apt install ros-$ROS_DISTRO-eigen3-cmake-module
sudo apt install libeigen3-dev
```

## 3. Create a ROS 2 Workspace
If you don't already have a colcon workspace, create one:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 4. Clone the Repository
Clone the project directly into the `src` folder of your workspace:
```bash
git clone <repository-url> IMU-SVD-Calib-ROS
```

## 5. Build the Package
Navigate back to the root of your workspace and build the package using colcon:
```bash
cd ~/ros2_ws
colcon build --packages-select imu_svd_calib
```

## 6. Source the Environment
Before running the nodes, always source your workspace setup file:
```bash
source install/setup.bash
```
