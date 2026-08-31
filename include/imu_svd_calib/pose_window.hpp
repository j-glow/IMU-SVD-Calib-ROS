#ifndef IMU_SVD_CALIB__POSE_WINDOW_HPP_
#define IMU_SVD_CALIB__POSE_WINDOW_HPP_

#include <Eigen/Dense>
#include <array>
#include <vector>

namespace imu_svd_calib
{

/**
 * @brief Check whether a window of raw samples corresponds to a single
 *        stable orientation rather than a blend across a pose transition.
 *
 * The calibration algorithms assume every column of the measurement matrix
 * satisfies ||v_j|| = c for a single fixed orientation v_j. Averaging
 * samples straddling a pose switch violates that assumption and silently
 * corrupts the calibration. A window is considered stable if every sample
 * in it lies within max_deviation of the window's own average.
 */
inline bool is_window_stable(
  const std::vector<std::array<double, 3>> & samples,
  const Eigen::Vector3d & window_average,
  double max_deviation)
{
  for (const auto & sample : samples) {
    Eigen::Vector3d s(sample[0], sample[1], sample[2]);
    if ((s - window_average).norm() > max_deviation) {
      return false;
    }
  }
  return true;
}

}  // namespace imu_svd_calib

#endif  // IMU_SVD_CALIB__POSE_WINDOW_HPP_
