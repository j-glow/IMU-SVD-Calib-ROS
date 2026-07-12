#ifndef IMU_SVD_CALIB__LM_CALIBRATION_HPP_
#define IMU_SVD_CALIB__LM_CALIBRATION_HPP_

#include <Eigen/Dense>
#include <vector>

namespace imu_svd_calib
{

/**
 * @brief Implementation of the classical Iterative (Levenberg-Marquardt) 
 *        calibration algorithm used as a baseline for comparison.
 */
class LmCalibration
{
public:
  /**
   * @brief Calibrate IMU sensors using the LM algorithm.
   * 
   * @param measurements Matrix M of size m x n where m is the number of sensor axes (e.g., 3) 
   *                     and n is the number of poses.
   * @param reference_magnitude The magnitude of the measured vector quantity (e.g., 9.81 for gravity).
   * @param initial_guess_matrix Initial guess for the calibration matrix.
   * @param initial_guess_bias Initial guess for the bias vector.
   * @param out_calibration_matrix Computed scaling and misalignment matrix.
   * @param out_bias_vector Computed bias vector.
   * @return true if calibration succeeded and converged, false otherwise.
   */
  static bool calibrate(
    const Eigen::MatrixXd& measurements,
    double reference_magnitude,
    const Eigen::MatrixXd& initial_guess_matrix,
    const Eigen::VectorXd& initial_guess_bias,
    Eigen::MatrixXd& out_calibration_matrix,
    Eigen::VectorXd& out_bias_vector);
};

} // namespace imu_svd_calib

#endif // IMU_SVD_CALIB__LM_CALIBRATION_HPP_
