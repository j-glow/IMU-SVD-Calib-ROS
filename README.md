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

The calibration node exposes a `calibrate_imu` ROS 2 action (`imu_svd_calib/action/CalibrateImu`)
instead of calibrating automatically on startup. Sending a goal starts data collection; feedback
reports each distinct pose as it is collected, and the result carries both the SVD and LM
calibration matrices, bias vectors, and RMSE for direct comparison. Internally, the node discards
any sample window that straddles a pose transition, detects distinct static poses, and once
`num_poses` have been collected (minimum 9, required by the SVD method) runs both calibration
algorithms and starts publishing raw vs. SVD-corrected vs. LM-corrected acceleration markers on
`/imu_visualization` for RViz.

A goal is rejected if a calibration is already in progress or `num_poses` is below 9. A goal can
be canceled mid-collection with `ros2 action cancel_goals /calibrate_imu` or Ctrl+C on the
`send_goal` client.

For reproducible comparative runs, the same data stream can also be recorded to a rosbag2 dataset
and replayed later instead of depending on a live simulator, removing timing jitter and simulator
randomness as a variable between runs.

Unit tests cover the SVD and LM calibration algorithms against known ground truth, and the
pose-window stability check that filters out samples straddling a pose transition.

**All terminal commands** — building, running the tests, running the live pipeline, the rosbag2
record/replay workflow, and RViz — are in the [Validation Guide](VALIDATION.md), along with the
expected output for each step.

