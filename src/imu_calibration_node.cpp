#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include "imu_svd_calib/svd_calibration.hpp"
#include "imu_svd_calib/lm_calibration.hpp"

using std::placeholders::_1;

class ImuCalibrationNode : public rclcpp::Node
{
public:
  ImuCalibrationNode() : Node("imu_calibration_node"), collecting_(true)
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10, std::bind(&ImuCalibrationNode::imu_callback, this, _1));
      
    RCLCPP_INFO(this->get_logger(), "IMU Calibration Node started. Waiting for poses...");
  }

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    Eigen::Vector3d raw(msg->linear_acceleration.x, 
                        msg->linear_acceleration.y, 
                        msg->linear_acceleration.z);

    if (!collecting_) {
      publish_visualization(raw);
      return;
    }

    current_pose_samples_.push_back({raw(0), raw(1), raw(2)});
                                     
    if (current_pose_samples_.size() == 100) {
      // Average 100 samples
      Eigen::Vector3d avg_acc(0.0, 0.0, 0.0);
      for (const auto& sample : current_pose_samples_) {
        avg_acc(0) += sample[0];
        avg_acc(1) += sample[1];
        avg_acc(2) += sample[2];
      }
      avg_acc /= 100.0;
      
      current_pose_samples_.clear();
      
      // Check if it's a distinct pose (euclidean distance > 2.0 m/s^2)
      bool distinct = true;
      for (const auto& pose : collected_poses_) {
        if ((pose - avg_acc).norm() < 2.0) {
          distinct = false;
          break;
        }
      }
      
      if (distinct) {
        collected_poses_.push_back(avg_acc);
        RCLCPP_INFO(this->get_logger(), "Collected distinct pose %zu/9", collected_poses_.size());
        
        if (collected_poses_.size() == 9) {
          collecting_ = false;
          run_calibration();
        }
      }
    }
  }
  
  void run_calibration()
  {
    RCLCPP_INFO(this->get_logger(), "Starting Calibration...");
    Eigen::MatrixXd M(3, 9);
    for (int i = 0; i < 9; ++i) {
      M.col(i) = collected_poses_[i];
    }
    
    double ref_mag = 9.81;
    Eigen::MatrixXd svd_A;
    Eigen::VectorXd svd_o;
    
    // 1. SVD Calibration
    bool svd_success = imu_svd_calib::SvdCalibration::calibrate(M, ref_mag, svd_A, svd_o);
    
    if (svd_success) {
      RCLCPP_INFO(this->get_logger(), "--- SVD Calibration Results ---");
      std::cout << "A Matrix:\n" << svd_A << "\n\nBias Vector:\n" << svd_o << std::endl;
      
      double svd_rmse = compute_rmse(M, svd_A, svd_o, ref_mag);
      RCLCPP_INFO(this->get_logger(), "SVD RMSE: %f m/s^2", svd_rmse);
    } else {
      RCLCPP_ERROR(this->get_logger(), "SVD Calibration failed.");
    }
    
    // 2. LM Baseline Calibration
    Eigen::MatrixXd lm_A;
    Eigen::VectorXd lm_o;
    Eigen::MatrixXd guess_A = Eigen::MatrixXd::Identity(3, 3);
    Eigen::VectorXd guess_o = Eigen::VectorXd::Zero(3);
    
    bool lm_success = imu_svd_calib::LmCalibration::calibrate(M, ref_mag, guess_A, guess_o, lm_A, lm_o);
    
    if (lm_success) {
      RCLCPP_INFO(this->get_logger(), "--- LM Calibration Results ---");
      std::cout << "A Matrix:\n" << lm_A << "\n\nBias Vector:\n" << lm_o << std::endl;
      
      double lm_rmse = compute_rmse(M, lm_A, lm_o, ref_mag);
      RCLCPP_INFO(this->get_logger(), "LM RMSE: %f m/s^2", lm_rmse);
    } else {
      RCLCPP_ERROR(this->get_logger(), "LM Calibration failed.");
    }
    
    // Switch to visualization mode
    svd_A_ = svd_A;
    svd_o_ = svd_o;
    lm_A_ = lm_A;
    lm_o_ = lm_o;
    
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/imu_visualization", 10);
    RCLCPP_INFO(this->get_logger(), "Calibration complete. Now publishing RViz markers on /imu_visualization...");
  }
  
  double compute_rmse(const Eigen::MatrixXd& M, const Eigen::MatrixXd& A, const Eigen::VectorXd& o, double ref) {
    double sq_err = 0.0;
    Eigen::MatrixXd K = A.inverse().transpose(); // K = A^{-T}
    for (int i = 0; i < M.cols(); ++i) {
      double norm = (K * (M.col(i) - o)).norm();
      sq_err += std::pow(norm - ref, 2);
    }
    return std::sqrt(sq_err / M.cols());
  }

  void publish_visualization(const Eigen::Vector3d& raw) {
    if (!marker_pub_) return;

    visualization_msgs::msg::MarkerArray markers;

    // Helper lambda to create arrow markers
    auto create_arrow = [&](int id, const Eigen::Vector3d& vec, float r, float g, float b, const std::string& ns) {
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
      end.x = vec(0) / 9.81;
      end.y = vec(1) / 9.81;
      end.z = vec(2) / 9.81;
      m.points.push_back(end);
      
      m.scale.x = 0.05; // Shaft diameter
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

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  bool collecting_;
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
