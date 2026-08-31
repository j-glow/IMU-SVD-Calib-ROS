#include "imu_svd_calib/lm_calibration.hpp"
#include <iostream>
#include <cmath>

namespace imu_svd_calib
{

bool LmCalibration::calibrate(
  const Eigen::MatrixXd & measurements,
  double reference_magnitude,
  const Eigen::MatrixXd & initial_guess_matrix,
  const Eigen::VectorXd & initial_guess_bias,
  Eigen::MatrixXd & out_calibration_matrix,
  Eigen::VectorXd & out_bias_vector)
{
  int n = measurements.cols();
  if (n < 9) {return false;}

  Eigen::VectorXd x(9);
  // We optimize M = K^T K, where K = A^{-T}
  Eigen::MatrixXd K = initial_guess_matrix.inverse().transpose();
  Eigen::MatrixXd M = K.transpose() * K;
  x << M(0, 0), M(0, 1), M(0, 2), M(1, 1), M(1, 2), M(2, 2),
    initial_guess_bias(0), initial_guess_bias(1), initial_guess_bias(2);

  double lambda = 1e-3;
  double c2 = reference_magnitude * reference_magnitude;

  for (int iter = 0; iter < 100; ++iter) {
    Eigen::VectorXd f(n);
    Eigen::MatrixXd J(n, 9);
    for (int j = 0; j < n; ++j) {
      Eigen::Vector3d m_v = measurements.col(j);
      Eigen::Vector3d o(x(6), x(7), x(8));
      Eigen::Matrix3d M_mat;
      M_mat << x(0), x(1), x(2),
        x(1), x(3), x(4),
        x(2), x(4), x(5);
      Eigen::Vector3d diff = m_v - o;
      f(j) = diff.transpose() * M_mat * diff - c2;

      J(j, 0) = diff(0) * diff(0);
      J(j, 1) = 2 * diff(0) * diff(1);
      J(j, 2) = 2 * diff(0) * diff(2);
      J(j, 3) = diff(1) * diff(1);
      J(j, 4) = 2 * diff(1) * diff(2);
      J(j, 5) = diff(2) * diff(2);

      Eigen::Vector3d Jo = -2 * M_mat * diff;
      J(j, 6) = Jo(0);
      J(j, 7) = Jo(1);
      J(j, 8) = Jo(2);
    }

    Eigen::MatrixXd JtJ = J.transpose() * J;
    Eigen::MatrixXd A_lm = JtJ + lambda * Eigen::MatrixXd::Identity(9, 9);
    Eigen::VectorXd g = J.transpose() * f;
    Eigen::VectorXd delta = A_lm.ldlt().solve(-g);

    if (delta.norm() < 1e-6) {break;}
    x += delta;
  }

  Eigen::Matrix3d M_final;
  M_final << x(0), x(1), x(2),
    x(1), x(3), x(4),
    x(2), x(4), x(5);

  // Recover K using Cholesky decomposition
  Eigen::LLT<Eigen::Matrix3d> llt(M_final);
  Eigen::Matrix3d K_final = llt.matrixU(); // upper triangular

  // A = K^{-T}
  out_calibration_matrix = K_final.inverse().transpose();
  out_bias_vector = Eigen::Vector3d(x(6), x(7), x(8));

  return true;
}

} // namespace imu_svd_calib
