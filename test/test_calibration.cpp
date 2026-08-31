#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <random>

#include "imu_svd_calib/svd_calibration.hpp"
#include "imu_svd_calib/lm_calibration.hpp"

using namespace imu_svd_calib;

namespace
{

constexpr double kRefMagnitude = 9.81;

// Self-consistency RMSE: does (A, o) explain the measurements with the
// correct reference magnitude? This does not require knowledge of the
// ground truth and is what imu_calibration_node reports at runtime.
double compute_rmse(
  const Eigen::MatrixXd & M, const Eigen::MatrixXd & A, const Eigen::VectorXd & o, double c)
{
  Eigen::MatrixXd K = A.inverse().transpose();
  double sq = 0.0;
  for (int j = 0; j < M.cols(); ++j) {
    double norm = (K * (M.col(j) - o)).norm();
    sq += (norm - c) * (norm - c);
  }
  return std::sqrt(sq / M.cols());
}

// Reconstruction error of A up to the unknown orthogonal transform L that
// Theorem 1 in Rupniewski (2025) says the algorithm can only recover A to
// (sigma_A, computed via the orthogonal Procrustes solution, eq. 12-13).
double procrustes_error_A(const Eigen::MatrixXd & A_true, const Eigen::MatrixXd & A_hat)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    A_hat * A_true.transpose(), Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd L = svd.matrixV() * svd.matrixU().transpose();
  Eigen::MatrixXd A_aligned = L * A_hat;
  double sq = (A_aligned - A_true).squaredNorm();
  return std::sqrt(sq / (A_true.rows() * A_true.cols()));
}

Eigen::MatrixXd nine_canonical_poses()
{
  double gs = std::sqrt(0.5);
  std::vector<Eigen::Vector3d> dirs = {
    {0, 0, 1}, {0, 0, -1}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
    {gs, gs, 0}, {gs, 0, gs}, {0, gs, gs}
  };
  Eigen::MatrixXd V(3, dirs.size());
  for (size_t j = 0; j < dirs.size(); ++j) {
    V.col(static_cast<int>(j)) = dirs[j] * kRefMagnitude;
  }
  return V;
}

// Ground truth calibration with non-trivial gains, misalignment, and bias,
// matching the measurement model M = A^T V + o*1^T (+ noise) from eq. 2/5.
struct GroundTruth
{
  Eigen::MatrixXd A;
  Eigen::VectorXd o;
};

GroundTruth sample_ground_truth()
{
  Eigen::MatrixXd A(3, 3);
  A << 1.02, 0.02, -0.01,
    0.01, 0.98, 0.005,
    -0.005, 0.015, 1.05;
  Eigen::VectorXd o(3);
  o << 0.15, -0.08, 0.05;
  return {A, o};
}

Eigen::MatrixXd measurements_from(
  const GroundTruth & gt, const Eigen::MatrixXd & V, double noise_std = 0.0, unsigned seed = 42)
{
  Eigen::MatrixXd M = gt.A.transpose() * V +
    gt.o * Eigen::VectorXd::Ones(V.cols()).transpose();
  if (noise_std > 0.0) {
    std::default_random_engine gen(seed);
    std::normal_distribution<double> noise(0.0, noise_std);
    for (int i = 0; i < M.rows(); ++i) {
      for (int j = 0; j < M.cols(); ++j) {
        M(i, j) += noise(gen);
      }
    }
  }
  return M;
}

}  // namespace

TEST(SvdCalibration, RecoversGroundTruthExactlyOnCleanData)
{
  GroundTruth gt = sample_ground_truth();
  Eigen::MatrixXd V = nine_canonical_poses();
  Eigen::MatrixXd M = measurements_from(gt, V);

  Eigen::MatrixXd A_hat;
  Eigen::VectorXd o_hat;
  ASSERT_TRUE(SvdCalibration::calibrate(M, kRefMagnitude, A_hat, o_hat));

  // Theorem 1: the bias has no orthogonal ambiguity and should be exact.
  EXPECT_NEAR((o_hat - gt.o).norm(), 0.0, 1e-6);
  EXPECT_NEAR(compute_rmse(M, A_hat, o_hat, kRefMagnitude), 0.0, 1e-6);
  EXPECT_NEAR(procrustes_error_A(gt.A, A_hat), 0.0, 1e-6);
}

TEST(SvdCalibration, StaysCloseToGroundTruthUnderNoise)
{
  GroundTruth gt = sample_ground_truth();
  Eigen::MatrixXd V = nine_canonical_poses();
  Eigen::MatrixXd M = measurements_from(gt, V, /*noise_std=*/0.1);

  Eigen::MatrixXd A_hat;
  Eigen::VectorXd o_hat;
  ASSERT_TRUE(SvdCalibration::calibrate(M, kRefMagnitude, A_hat, o_hat));

  // Corollary 2 (stability): small noise should yield a small, bounded
  // reconstruction error, not the multiple-orders-of-magnitude blow-up seen
  // when a pose is contaminated by a transition (see the regression test
  // below and imu_calibration_node's window-stability check).
  EXPECT_LT(compute_rmse(M, A_hat, o_hat, kRefMagnitude), 1.0);
  EXPECT_LT((o_hat - gt.o).norm(), 1.0);
}

TEST(LmCalibration, RecoversGroundTruthExactlyOnCleanData)
{
  GroundTruth gt = sample_ground_truth();
  Eigen::MatrixXd V = nine_canonical_poses();
  Eigen::MatrixXd M = measurements_from(gt, V);

  Eigen::MatrixXd A_hat;
  Eigen::VectorXd o_hat;
  Eigen::MatrixXd guess_A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd guess_o = Eigen::VectorXd::Zero(3);
  ASSERT_TRUE(LmCalibration::calibrate(M, kRefMagnitude, guess_A, guess_o, A_hat, o_hat));

  EXPECT_NEAR((o_hat - gt.o).norm(), 0.0, 1e-6);
  EXPECT_NEAR(compute_rmse(M, A_hat, o_hat, kRefMagnitude), 0.0, 1e-6);
  EXPECT_NEAR(procrustes_error_A(gt.A, A_hat), 0.0, 1e-6);
}

TEST(LmCalibration, StaysCloseToGroundTruthUnderNoise)
{
  GroundTruth gt = sample_ground_truth();
  Eigen::MatrixXd V = nine_canonical_poses();
  Eigen::MatrixXd M = measurements_from(gt, V, /*noise_std=*/0.1);

  Eigen::MatrixXd A_hat;
  Eigen::VectorXd o_hat;
  Eigen::MatrixXd guess_A = Eigen::MatrixXd::Identity(3, 3);
  Eigen::VectorXd guess_o = Eigen::VectorXd::Zero(3);
  ASSERT_TRUE(LmCalibration::calibrate(M, kRefMagnitude, guess_A, guess_o, A_hat, o_hat));

  EXPECT_LT(compute_rmse(M, A_hat, o_hat, kRefMagnitude), 1.0);
  EXPECT_LT((o_hat - gt.o).norm(), 1.0);
}

// Regression test for the pose-averaging bug fixed in imu_calibration_node:
// a measurement matrix where one pose is a blend across a transition (its
// column norm is far from kRefMagnitude) breaks the ||v||=c assumption both
// algorithms rely on and should produce a visibly large RMSE. This is the
// exact matrix captured from a real (pre-fix) run of the node.
TEST(SvdCalibration, ContaminatedPoseProducesLargeRmseNotSilentCorruption)
{
  Eigen::MatrixXd M(3, 9);
  M << 0.009109469196, -0.009274484478, -0.005953216723, 4.51927781,
       9.799411817, -9.807022858, -5.314358745, 0.01367571553, -0.01635779573,
      -0.004642771795, 0.002136238742, 0.02446722759, -0.01232836085,
       0.0002386900745, 0.0118020894, 4.503432445, 9.821734928, -9.821104473,
       9.798467912, 0.779125597, -9.806333742, -5.300643368, 0.003324058302,
      -0.003469551004, 0.009437491509, -0.01211940146, -0.0009747068541;

  Eigen::MatrixXd A_hat;
  Eigen::VectorXd o_hat;
  ASSERT_TRUE(SvdCalibration::calibrate(M, kRefMagnitude, A_hat, o_hat));

  // Document the failure mode rather than asserting a specific value: a
  // contaminated pose should be clearly detectable via RMSE, motivating the
  // window-stability check that now prevents such poses from ever reaching
  // the algorithm.
  EXPECT_GT(compute_rmse(M, A_hat, o_hat, kRefMagnitude), 10.0);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
