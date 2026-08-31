#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <array>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "imu_svd_calib/action/calibrate_imu.hpp"
#include "imu_svd_calib/svd_calibration.hpp"
#include "imu_svd_calib/lm_calibration.hpp"
#include "imu_svd_calib/pose_window.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
std::array<double, 9> flatten_row_major(const Eigen::Matrix3d & m)
{
  std::array<double, 9> out{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i * 3 + j] = m(i, j);
    }
  }
  return out;
}
}  // namespace

class ImuCalibrationNode : public rclcpp::Node
{
public:
  using CalibrateImu = imu_svd_calib::action::CalibrateImu;
  using GoalHandleCalibrateImu = rclcpp_action::ServerGoalHandle<CalibrateImu>;

  ImuCalibrationNode()
  : Node("imu_calibration_node")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&ImuCalibrationNode::imu_callback, this, _1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/imu_visualization", 10);

    action_server_ = rclcpp_action::create_server<CalibrateImu>(
      this,
      "calibrate_imu",
      std::bind(&ImuCalibrationNode::handle_goal, this, _1, _2),
      std::bind(&ImuCalibrationNode::handle_cancel, this, _1),
      std::bind(&ImuCalibrationNode::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(),
      "IMU Calibration Node started. Send a calibrate_imu goal to begin.");
  }

private:
  // --- Action server callbacks ---

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const CalibrateImu::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: a calibration is already in progress.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->num_poses < 9) {
      RCLCPP_WARN(
        this->get_logger(), "Rejecting goal: num_poses must be >= 9, got %u.", goal->num_poses);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCalibrateImu>)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cancel_requested_ = true;
    cv_.notify_all();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      active_goal_ = goal_handle;
      num_poses_target_ = goal_handle->get_goal()->num_poses;
      current_pose_samples_.clear();
      collected_poses_.clear();
      cancel_requested_ = false;
      done_ = false;
      RCLCPP_INFO(
        this->get_logger(), "Calibration goal accepted. Collecting %u distinct poses...",
        num_poses_target_);
    }
    // Actions must not block the goal-accepted callback; the actual work
    // happens asynchronously in imu_callback as samples arrive, this thread
    // just waits for that to finish (or a cancel) and reports the outcome.
    std::thread{std::bind(&ImuCalibrationNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleCalibrateImu> goal_handle)
  {
    std::shared_ptr<CalibrateImu::Result> result;
    bool canceled;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this] {return done_ || cancel_requested_;});
      canceled = cancel_requested_;
      result = result_;
      active_goal_.reset();
    }

    if (canceled) {
      auto empty_result = std::make_shared<CalibrateImu::Result>();
      empty_result->success = false;
      goal_handle->canceled(empty_result);
      RCLCPP_INFO(this->get_logger(), "Calibration goal canceled.");
    } else {
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Calibration goal succeeded.");
    }
  }

  // --- IMU sample processing ---

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    Eigen::Vector3d raw(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    std::lock_guard<std::mutex> lock(mutex_);

    if (visualizing_) {
      publish_visualization(raw);
    }

    if (!active_goal_ || cancel_requested_) {
      return;
    }

    current_pose_samples_.push_back({raw(0), raw(1), raw(2)});
    if (current_pose_samples_.size() != kSamplesPerWindow) {
      return;
    }

    Eigen::Vector3d avg_acc(0.0, 0.0, 0.0);
    for (const auto & sample : current_pose_samples_) {
      avg_acc(0) += sample[0];
      avg_acc(1) += sample[1];
      avg_acc(2) += sample[2];
    }
    avg_acc /= static_cast<double>(kSamplesPerWindow);

    // Reject windows that straddle a pose transition (see is_window_stable):
    // the average would not correspond to a single stable orientation and
    // would violate the ||v||=c model assumption.
    bool stable = imu_svd_calib::is_window_stable(
      current_pose_samples_, avg_acc, kMaxWindowDeviation);
    current_pose_samples_.clear();
    if (!stable) {
      // Unstable window (mid-transition): discard and resynchronize on the
      // next batch rather than treating it as a valid pose.
      return;
    }

    bool distinct = true;
    for (const auto & pose : collected_poses_) {
      if ((pose - avg_acc).norm() < kDistinctPoseThreshold) {
        distinct = false;
        break;
      }
    }
    if (!distinct) {
      return;
    }

    collected_poses_.push_back(avg_acc);

    auto feedback = std::make_shared<CalibrateImu::Feedback>();
    feedback->poses_collected = static_cast<uint32_t>(collected_poses_.size());
    feedback->poses_needed = num_poses_target_;
    feedback->status = "Collected distinct pose " + std::to_string(collected_poses_.size()) +
      "/" + std::to_string(num_poses_target_);
    active_goal_->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "%s", feedback->status.c_str());

    if (collected_poses_.size() == num_poses_target_) {
      run_calibration();
      done_ = true;
      cv_.notify_all();
    }
  }

  // Runs both calibration methods and fills result_. Must be called with
  // mutex_ held (invoked only from imu_callback).
  void run_calibration()
  {
    RCLCPP_INFO(this->get_logger(), "Starting Calibration...");
    Eigen::MatrixXd M(3, num_poses_target_);
    for (uint32_t i = 0; i < num_poses_target_; ++i) {
      M.col(i) = collected_poses_[i];
    }

    auto result = std::make_shared<CalibrateImu::Result>();
    result->success = true;

    // 1. SVD Calibration
    Eigen::MatrixXd svd_A;
    Eigen::VectorXd svd_o;
    bool svd_success = imu_svd_calib::SvdCalibration::calibrate(M, kRefMagnitude, svd_A, svd_o);
    if (svd_success) {
      result->svd_calibration_matrix = flatten_row_major(svd_A);
      result->svd_bias_vector = {svd_o(0), svd_o(1), svd_o(2)};
      result->svd_rmse = compute_rmse(M, svd_A, svd_o, kRefMagnitude);
      RCLCPP_INFO(this->get_logger(), "SVD RMSE: %f m/s^2", result->svd_rmse);
      svd_A_ = svd_A;
      svd_o_ = svd_o;
    } else {
      RCLCPP_ERROR(this->get_logger(), "SVD Calibration failed.");
      result->success = false;
    }

    // 2. LM Baseline Calibration
    Eigen::MatrixXd lm_A;
    Eigen::VectorXd lm_o;
    Eigen::MatrixXd guess_A = Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd guess_o = Eigen::VectorXd::Zero(3);
    bool lm_success = imu_svd_calib::LmCalibration::calibrate(
      M, kRefMagnitude, guess_A, guess_o, lm_A, lm_o);
    if (lm_success) {
      result->lm_calibration_matrix = flatten_row_major(lm_A);
      result->lm_bias_vector = {lm_o(0), lm_o(1), lm_o(2)};
      result->lm_rmse = compute_rmse(M, lm_A, lm_o, kRefMagnitude);
      RCLCPP_INFO(this->get_logger(), "LM RMSE: %f m/s^2", result->lm_rmse);
      lm_A_ = lm_A;
      lm_o_ = lm_o;
    } else {
      RCLCPP_ERROR(this->get_logger(), "LM Calibration failed.");
      result->success = false;
    }

    result_ = result;
    if (svd_success && lm_success) {
      visualizing_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Calibration complete. Publishing RViz markers on /imu_visualization...");
    }
  }

  double compute_rmse(
    const Eigen::MatrixXd & M, const Eigen::MatrixXd & A,
    const Eigen::VectorXd & o, double ref)
  {
    double sq_err = 0.0;
    Eigen::MatrixXd K = A.inverse().transpose();  // K = A^{-T}
    for (int i = 0; i < M.cols(); ++i) {
      double norm = (K * (M.col(i) - o)).norm();
      sq_err += std::pow(norm - ref, 2);
    }
    return std::sqrt(sq_err / M.cols());
  }

  // Must be called with mutex_ held.
  void publish_visualization(const Eigen::Vector3d & raw)
  {
    visualization_msgs::msg::MarkerArray markers;

    // Helper lambda to create arrow markers
    auto create_arrow = [&](int id, const Eigen::Vector3d & vec, float r, float g, float b,
      const std::string & ns) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "imu_link";
        m.header.stamp = this->now();
        m.ns = ns;
        m.id = id;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point start;
        start.x = 0; start.y = 0; start.z = 0;
        m.points.push_back(start);

        geometry_msgs::msg::Point end;
        // Scale down vector for visualization
        end.x = vec(0) / kRefMagnitude;
        end.y = vec(1) / kRefMagnitude;
        end.z = vec(2) / kRefMagnitude;
        m.points.push_back(end);

        m.scale.x = 0.05;  // Shaft diameter
        m.scale.y = 0.1;  // Head diameter
        m.scale.z = 0.1;  // Head length
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;
        return m;
      };

    // 1. Raw Vector (Red)
    markers.markers.push_back(create_arrow(0, raw, 1.0, 0.0, 0.0, "raw"));

    // 2. SVD Corrected (Green)
    Eigen::MatrixXd svd_K = svd_A_.inverse().transpose();
    Eigen::Vector3d svd_corrected = svd_K * (raw - svd_o_);
    markers.markers.push_back(create_arrow(1, svd_corrected, 0.0, 1.0, 0.0, "svd_corrected"));

    // 3. LM Corrected (Blue)
    Eigen::MatrixXd lm_K = lm_A_.inverse().transpose();
    Eigen::Vector3d lm_corrected = lm_K * (raw - lm_o_);
    markers.markers.push_back(create_arrow(2, lm_corrected, 0.0, 0.0, 1.0, "lm_corrected"));

    marker_pub_->publish(markers);
  }

  // Reference magnitude of the measured vector quantity (gravity, m/s^2).
  static constexpr double kRefMagnitude = 9.81;
  // Raw samples averaged per pose window (100 samples @ 100 Hz = ~1 s).
  static constexpr size_t kSamplesPerWindow = 100;
  // Max allowed per-sample deviation (m/s^2) from a window's average before
  // the window is rejected as straddling a pose transition. Well above the
  // simulator's 0.1 m/s^2 noise floor, well below a real pose change.
  static constexpr double kMaxWindowDeviation = 1.0;
  // Minimum euclidean distance (m/s^2) between averaged readings for a new
  // window to be considered a distinct pose.
  static constexpr double kDistinctPoseThreshold = 2.0;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp_action::Server<CalibrateImu>::SharedPtr action_server_;

  std::mutex mutex_;
  std::condition_variable cv_;
  std::shared_ptr<GoalHandleCalibrateImu> active_goal_;
  std::shared_ptr<CalibrateImu::Result> result_;
  uint32_t num_poses_target_ = 0;
  bool cancel_requested_ = false;
  bool done_ = false;
  bool visualizing_ = false;

  std::vector<std::array<double, 3>> current_pose_samples_;
  std::vector<Eigen::Vector3d> collected_poses_;

  Eigen::MatrixXd svd_A_;
  Eigen::VectorXd svd_o_;
  Eigen::MatrixXd lm_A_;
  Eigen::VectorXd lm_o_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
