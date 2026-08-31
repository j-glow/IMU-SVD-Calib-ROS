# Validation Guide

A step-by-step walkthrough for manually verifying that the package builds, tests pass, and the
calibration pipeline produces sane results. Assumes the workspace is already set up per the
[Full Setup Guide](SETUP_GUIDE.md).

## 1. Build (with tests enabled)

```bash
source /opt/ros/lyrical/setup.bash
cd ~/ros2_ws
colcon build --packages-select imu_svd_calib --cmake-args -DBUILD_TESTING=ON
```

**Expect:** `Finished <<< imu_svd_calib`, no errors. First build takes roughly 10-25 seconds.

## 2. Run the automated test suite

The fastest, most objective check — no live nodes or timing to wait on.

```bash
source ~/ros2_ws/install/setup.bash
colcon test --packages-select imu_svd_calib
colcon test-result --verbose
```

**Expect:** `Summary: 41 tests, 0 errors, 0 failures, 9 skipped`. The 9 skipped are lint checks that
only run under `--event-handlers console_direct+`; safe to ignore. Any `failures > 0` is a real
regression in `SvdCalibration`, `LmCalibration`, or the pose-window stability check.

## 3. Run the live pipeline end-to-end

Three terminals, each sourcing the workspace first (`source ~/ros2_ws/install/setup.bash`):

**Terminal 1:**
```bash
ros2 run imu_svd_calib imu_simulator_node
```

**Terminal 2** (start a couple seconds after terminal 1):
```bash
ros2 run imu_svd_calib imu_calibration_node
```

**Terminal 3:**
```bash
ros2 action send_goal /calibrate_imu imu_svd_calib/action/CalibrateImu "{num_poses: 9}" --feedback
```

**Expect:** feedback lines counting `1/9` up to `9/9` over roughly 40-50 seconds (the simulator
switches pose every 4s; a window is silently discarded and re-collected if it straddles a pose
switch, so the count doesn't always advance on a fixed cadence). Then a `Result` block with
`success: true` and both `svd_rmse` and `lm_rmse` **extremely close to zero** (order of `1e-10` to
`1e-14` — floating-point noise, not a meaningfully large number). RMSE in the tens or hundreds
indicates a broken pose (e.g. a reintroduced version of the window-straddling bug fixed in
`imu_calibration_node`) — see `test_calibration.cpp`'s `ContaminatedPoseProducesLargeRmse...` test
for the historical failure mode this guards against.

## 4. Validate the rosbag2 workflow

Worth doing once, since this is the reproducibility mechanism intended for the thesis's
comparative experiments (ch. 4).

```bash
ros2 launch imu_svd_calib record_dataset.launch.py bag_path:=/tmp/imu_test_dataset
```
Let it run ~50 seconds (through at least one full pose cycle), then Ctrl+C. Then:
```bash
ros2 launch imu_svd_calib playback_calibration.launch.py bag_path:=/tmp/imu_test_dataset
```
and in another terminal, send the same `send_goal` command as step 3.

**Expect:** the same kind of result as step 3 (RMSE near zero), now produced from replayed data
instead of a live simulator — confirming the calibration node behaves identically either way.

## 5. RViz visualization

```bash
ros2 launch imu_svd_calib visualization.launch.py
```
then send a goal as in step 3.
