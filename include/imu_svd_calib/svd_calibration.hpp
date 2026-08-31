#ifndef IMU_SVD_CALIB__SVD_CALIBRATION_HPP_
#define IMU_SVD_CALIB__SVD_CALIBRATION_HPP_

#include <Eigen/Dense>
#include <vector>
#include <stdexcept>

namespace imu_svd_calib
{

/**
 * @brief Implementation of the algebraic SVD calibration algorithm
 *        proposed by Marek W. Rupniewski.
 */
class SvdCalibration
{
public:
  /**
   * @brief Calibrate IMU sensors using the SVD algorithm.
   *
   * @param measurements Matrix M of size m x n where m is the number of sensor axes (e.g., 3)
   *                     and n is the number of poses. n must be >= 9 for 3D space.
   * @param reference_magnitude The magnitude of the measured vector quantity (e.g., 9.81 for gravity).
   * @param out_calibration_matrix Computed scaling and misalignment matrix (A_hat).
   * @param out_bias_vector Computed bias vector (o_hat).
   * @return true if calibration succeeded, false otherwise.
   */
  static bool calibrate(
    const Eigen::MatrixXd & measurements,
    double reference_magnitude,
    Eigen::MatrixXd & out_calibration_matrix,
    Eigen::VectorXd & out_bias_vector);
};

} // namespace imu_svd_calib

#endif // IMU_SVD_CALIB__SVD_CALIBRATION_HPP_
