#include <gtest/gtest.h>
#include "imu_svd_calib/pose_window.hpp"

using imu_svd_calib::is_window_stable;

TEST(PoseWindow, StableWindowIsAccepted)
{
  // All samples close to a single orientation, within noise.
  std::vector<std::array<double, 3>> samples = {
    {0.0, 0.0, 9.81}, {0.01, -0.01, 9.80}, {-0.02, 0.0, 9.82}, {0.0, 0.02, 9.79}
  };
  Eigen::Vector3d avg(0.0, 0.0, 9.805);
  EXPECT_TRUE(is_window_stable(samples, avg, 1.0));
}

TEST(PoseWindow, WindowStraddlingATransitionIsRejected)
{
  // Half the samples at one pose, half at another: the average is a blend
  // that does not correspond to any single stable orientation.
  std::vector<std::array<double, 3>> samples = {
    {0.0, 0.0, 9.81}, {0.0, 0.0, 9.81}, {9.81, 0.0, 0.0}, {9.81, 0.0, 0.0}
  };
  Eigen::Vector3d avg(4.905, 0.0, 4.905);
  EXPECT_FALSE(is_window_stable(samples, avg, 1.0));
}

TEST(PoseWindow, DeviationExactlyAtThresholdIsAccepted)
{
  std::vector<std::array<double, 3>> samples = {{1.0, 0.0, 0.0}};
  Eigen::Vector3d avg(0.0, 0.0, 0.0);
  EXPECT_TRUE(is_window_stable(samples, avg, 1.0));
}

TEST(PoseWindow, EmptyWindowIsTriviallyStable)
{
  std::vector<std::array<double, 3>> samples;
  Eigen::Vector3d avg(0.0, 0.0, 9.81);
  EXPECT_TRUE(is_window_stable(samples, avg, 1.0));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
