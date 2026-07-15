# Full Setup Guide

This guide will walk you through the complete setup process for the IMU-SVD-Calib-ROS package.

Now that your APT package sources are clean and conflict-free, you are ready to install ROS 2 and configure your environment. On Ubuntu 26.04 ("Resolute"), the official long-term support (LTS) release of ROS 2 is Lyrical Luth. Here is the complete walkthrough to install it, set up your workspace tools, and verify the environment.

## 1. Install ROS 2 Lyrical Luth
Includes a workaround for a known Ubuntu 26.04 bug. There is a known packaging bug on Ubuntu 26.04 where an upstream package dependency (hyperspec) fails to install due to a dead server, which can block your entire ROS install. Prevent this error by explicitly excluding hyperspec during the desktop package installation:
```bash
sudo apt install ros-lyrical-desktop hyperspec- -y
```

## 2. Initialize and update rosdep
Manages system dependencies for your workspaces. The rosdep tool resolves system dependencies for any source code you compile. Run these commands to initialize and update its database:
```bash
sudo rosdep init
rosdep update
```
Note: If `sudo rosdep init` tells you the file already exists, you can safely ignore the warning and just run `rosdep update` as a normal (non-root) user.

## 3. Source the ROS 2 environment
Allows your terminal to recognize ROS commands. To activate ROS 2 in your current terminal session, source the setup script:
```bash
source /opt/ros/lyrical/setup.bash
```
To automatically source ROS 2 every time you open a new terminal, append the command to your `~/.bashrc` file:
```bash
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
```

## 4. Verify your environment
Double-check the active ROS variables. Run this quick check to ensure the ROS environment variables are populated correctly:
```bash
printenv | grep ROS
```
If successful, you should see output confirming the setup:
```plaintext
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=lyrical
```

## 5. Install Dependencies
You will need a C++17 compatible compiler and Eigen3.
On Ubuntu 26.04, you can install the necessary dependencies via:
```bash
sudo apt update
sudo apt install build-essential cmake
sudo apt install ros-$ROS_DISTRO-eigen3-cmake-module
sudo apt install libeigen3-dev
```

## 6. Create a ROS 2 Workspace
If you don't already have a colcon workspace, create one:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

## 7. Clone the Repository
Clone the project directly into the `src` folder of your workspace:
```bash
git clone https://github.com/j-glow/IMU-SVD-Calib-ROS.git IMU-SVD-Calib-ROS
```

## 8. Build the Package
Navigate back to the root of your workspace and build the package using colcon:
```bash
cd ~/ros2_ws
colcon build --packages-select imu_svd_calib
```

## 9. Source the Environment
Before running the nodes, always source your workspace setup file:
```bash
source install/setup.bash
```
