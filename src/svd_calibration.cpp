#include "imu_svd_calib/svd_calibration.hpp"
#include <iostream>
#include <cmath>

namespace imu_svd_calib
{

bool SvdCalibration::calibrate(
  const Eigen::MatrixXd & measurements,
  double reference_magnitude,
  Eigen::MatrixXd & out_calibration_matrix,
  Eigen::VectorXd & out_bias_vector)
{
  int m = measurements.rows(); // Number of sensor axes (e.g., 3)
  int n = measurements.cols(); // Number of poses (e.g., >= 9)

  if (n < 9 || m < 3) {
    std::cerr << "SVD Calibration requires at least 9 poses and 3 sensor axes for 3D." << std::endl;
    return false;
  }

  int d = m; // Assume d = m for a 3-axis IMU

  // Step 1: Compute rank-d truncated SVD of centered M
  Eigen::VectorXd mean_m = measurements.rowwise().mean();
  Eigen::MatrixXd M_centered = measurements.colwise() - mean_m;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_centered, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd U = svd.matrixU().leftCols(d);
  Eigen::MatrixXd S = svd.singularValues().head(d).asDiagonal();
  Eigen::MatrixXd W = svd.matrixV().leftCols(d);

  // Step 2: Formulate and solve the linear system for symmetric matrix H
  Eigen::VectorXd ones_d = Eigen::VectorXd::Ones(d);
  Eigen::VectorXd ones_n = Eigen::VectorXd::Ones(n);

  Eigen::MatrixXd B_trans(n, d + 1);
  B_trans << W, (ones_n - W * ones_d);
  Eigen::MatrixXd B = B_trans.transpose(); // (d+1) x n

  Eigen::MatrixXd A_sys(n + 1, 10);
  Eigen::VectorXd y_sys = Eigen::VectorXd::Zero(n + 1);

  for (int j = 0; j < n; ++j) {
    double b0 = B(0, j);
    double b1 = B(1, j);
    double b2 = B(2, j);
    double b3 = B(3, j);
    A_sys(j, 0) = b0 * b0;
    A_sys(j, 1) = 2 * b0 * b1;
    A_sys(j, 2) = 2 * b0 * b2;
    A_sys(j, 3) = 2 * b0 * b3;
    A_sys(j, 4) = b1 * b1;
    A_sys(j, 5) = 2 * b1 * b2;
    A_sys(j, 6) = 2 * b1 * b3;
    A_sys(j, 7) = b2 * b2;
    A_sys(j, 8) = 2 * b2 * b3;
    A_sys(j, 9) = b3 * b3;
    y_sys(j) = 1.0;
  }
  // trace H = 0 => h0 + h4 + h7 + h9 = 0
  A_sys(n, 0) = 1.0;
  A_sys(n, 4) = 1.0;
  A_sys(n, 7) = 1.0;
  A_sys(n, 9) = 1.0;

  Eigen::VectorXd h_vec = A_sys.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(y_sys);

  Eigen::MatrixXd H(4, 4);
  H << h_vec(0), h_vec(1), h_vec(2), h_vec(3),
    h_vec(1), h_vec(4), h_vec(5), h_vec(6),
    h_vec(2), h_vec(5), h_vec(7), h_vec(8),
    h_vec(3), h_vec(6), h_vec(8), h_vec(9);

  // Step 3: Generalized Eigenvalue problem G = G' - lambda_0 (H - G')
  Eigen::MatrixXd G_prime = Eigen::MatrixXd::Ones(4, 4);
  Eigen::MatrixXd H_minus_G_prime = H - G_prime;
  Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> ges(G_prime, H_minus_G_prime);

  double max_abs_val = -1.0;
  double lambda_0 = 0.0;
  for (int i = 0; i < 4; ++i) {
    double real_val = ges.eigenvalues()(i).real();
    if (std::abs(real_val) > max_abs_val) {
      max_abs_val = std::abs(real_val);
      lambda_0 = real_val;
    }
  }
  Eigen::MatrixXd G = G_prime - lambda_0 * H_minus_G_prime;

  // Step 4: Compact SVD of G
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(G);
  Eigen::VectorXd evals = eig.eigenvalues().tail(3); // top 3 eigenvalues
  Eigen::MatrixXd evecs = eig.eigenvectors().rightCols(3); // corresponding eigenvectors

  Eigen::MatrixXd Q = evecs.transpose(); // 3 x 4
  Eigen::MatrixXd Lambda_sqrt = evals.cwiseMax(0.0).cwiseSqrt().asDiagonal();
  Eigen::MatrixXd F = reference_magnitude * Lambda_sqrt * Q; // 3 x 4

  // Step 5: Find A_hat and o_hat
  Eigen::MatrixXd S_0_under = Eigen::MatrixXd::Ones(4, 4);
  S_0_under.topLeftCorner(3, 3) = S;
  S_0_under.topRightCorner(3, 1) = Eigen::VectorXd::Zero(3);

  Eigen::MatrixXd Um(3, 4);
  Um << U, mean_m;
  Eigen::MatrixXd E = Um * S_0_under; // 3 x 4

  Eigen::MatrixXd F_under = Eigen::MatrixXd::Ones(4, 4);
  F_under.topRows(3) = F;

  Eigen::MatrixXd Ao = E * F_under.inverse(); // 3 x 4

  out_calibration_matrix = Ao.leftCols(3).transpose();
  out_bias_vector = Ao.col(3);

  return true;
}

} // namespace imu_svd_calib
