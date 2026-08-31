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

Once set up, see the [Validation Guide](VALIDATION.md) for a step-by-step walkthrough of building,
testing, and running the pipeline with expected outputs for each step.

## Running the Project

The calibration node exposes a `calibrate_imu` ROS 2 action (`imu_svd_calib/action/CalibrateImu`)
instead of calibrating automatically on startup. Sending a goal starts data collection; feedback
reports each distinct pose as it is collected, and the result carries both the SVD and LM
calibration matrices, bias vectors, and RMSE for direct comparison.

1. Open a terminal, source the workspace, and run the simulator node (which will simulate the IMU
   orientations over time):
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run imu_svd_calib imu_simulator_node
   ```

2. Open a second terminal, source the workspace, and run the calibration node:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 run imu_svd_calib imu_calibration_node
   ```

3. Open a third terminal and send a calibration goal, requesting at least 9 distinct poses:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ros2 action send_goal /calibrate_imu imu_svd_calib/action/CalibrateImu "{num_poses: 9}" --feedback
   ```
   The calibration node collects samples, discards any window that straddles a pose transition,
   detects distinct static poses, and once `num_poses` have been collected runs both the SVD and
   LM calibration algorithms. The result includes both methods' calibration matrices, bias
   vectors, and RMSE. After the goal succeeds, the node also starts publishing raw vs.
   SVD-corrected vs. LM-corrected acceleration markers on `/imu_visualization` for RViz.

A goal is rejected if a calibration is already in progress, or if `num_poses` is below 9 (the
minimum required by the SVD method). A goal can be canceled mid-collection with
`ros2 action cancel_goals /calibrate_imu` or Ctrl+C on the `send_goal` client.

### Recording and replaying datasets with rosbag2

To make comparative runs reproducible, the simulated data stream can be recorded to a rosbag2
dataset and replayed later instead of depending on a live simulator:

```bash
# Record ~40+ seconds so the pose cycle completes at least once
ros2 launch imu_svd_calib record_dataset.launch.py bag_path:=/path/to/dataset

# Later, replay the exact same dataset and calibrate against it
ros2 launch imu_svd_calib playback_calibration.launch.py bag_path:=/path/to/dataset
# in another terminal:
ros2 action send_goal /calibrate_imu imu_svd_calib/action/CalibrateImu "{num_poses: 9}" --feedback
```

Processing the same recorded file removes timing jitter and simulator randomness as a variable,
so repeated algorithm runs (or runs after a code change) are directly comparable.

### RViz visualization

```bash
ros2 launch imu_svd_calib visualization.launch.py
```
launches the simulator, calibration node, and RViz together (still requires sending a
`calibrate_imu` goal to trigger calibration and start the marker publishing).

## Testing

Unit tests cover the SVD and LM calibration algorithms against known ground truth, and the
pose-window stability check that filters out samples straddling a pose transition:

```bash
colcon test --packages-select imu_svd_calib
colcon test-result --verbose
```

