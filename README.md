# IMU-SVD-Calib-ROS

ROS 2 package for IMU calibration focusing on a deterministic **Algebraic Method (SVD)** based on Rupniewski's research, alongside a classical **Iterative Method (Levenberg-Marquardt)** for performance comparison.

## Theoretical Background

### Main Algorithm: SVD Calibration (Rupniewski)
The primary method evaluated in this project is the novel algebraic solution for multi-pose IMU calibration proposed by Marek W. Rupniewski. This method infers the direction of the measured vector quantity directly from collected data using Singular Value Decomposition (SVD) and generalized eigenvalue problems. 

**Key Advantages:**
- **Deterministic:** The algorithm requires a fixed number of non-iterative steps, completely bypassing non-linear optimization.
- **No Initial Guess Required:** Unlike iterative methods, it avoids issues with local minima by not requiring an initial parameter guess.
- **Efficiency:** The runtime is predictable and it analytically solves for calibration parameters (gains, biases, and relative orientations).
- **Minimum Poses:** Requires at least 9 distinct static orientations for a full 3D calibration.

### Baseline Comparison: Iterative Method (Levenberg-Marquardt)
To comprehensively evaluate the SVD method, we implement a classical multi-pose calibration approach based on the Levenberg-Marquardt (LM) algorithm. The LM method minimizes a residual error cost function through non-linear optimization. While effective, it typically requires a good initial guess and its convergence speed can vary.

## Setup and Installation

For a comprehensive, step-by-step walkthrough on how to install ROS 2, configure dependencies (like Eigen3), and build this workspace from scratch, please see the [Full Setup Guide](SETUP_GUIDE.md).

## Running the Project
 
 1. Open a terminal, source the workspace, and run the simulator node (which will simulate the IMU orientations over time):
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run imu_svd_calib imu_simulator_node
    ```
 
 2. Open a second terminal, source the workspace, and run the calibration node:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run imu_svd_calib imu_calibration_node
    ```
 
 3. The calibration node will subscribe to `/imu/data`. It will collect samples, detect 9 distinct static poses automatically, and then execute both the SVD and LM calibration algorithms. The computed calibration matrices, biases, and RMSE statistics will be printed directly to the terminal for comparison.

