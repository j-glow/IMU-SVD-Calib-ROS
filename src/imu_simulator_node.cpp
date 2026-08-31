#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <array>
#include <chrono>

using namespace std::chrono_literals;

class ImuSimulatorNode : public rclcpp::Node
{
public:
  ImuSimulatorNode()
  : Node("imu_simulator_node"), current_pose_index_(0)
  {
    // Initialize standard poses (gravity vectors in sensor frame)
    double g = 9.81;
    double g_sqrt2 = g * std::sqrt(0.5);

    poses_ = {
      {0.0, 0.0, g},
      {0.0, 0.0, -g},
      {g, 0.0, 0.0},
      {-g, 0.0, 0.0},
      {0.0, g, 0.0},
      {0.0, -g, 0.0},
      {g_sqrt2, g_sqrt2, 0.0},
      {g_sqrt2, 0.0, g_sqrt2},
      {0.0, g_sqrt2, g_sqrt2}
    };

    // Publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

    // Random noise generators
    std::random_device rd;
    generator_ = std::default_random_engine(rd());
    accel_noise_dist_ = std::normal_distribution<double>(0.0, 0.1); // 0.1 m/s^2 std dev
    gyro_noise_dist_ = std::normal_distribution<double>(0.0, 0.01); // 0.01 rad/s std dev

    // Timers
    // 100 Hz publishing rate
    publish_timer_ = this->create_wall_timer(
      10ms, std::bind(&ImuSimulatorNode::publish_data, this));

    // Switch pose every 4 seconds
    pose_switch_timer_ = this->create_wall_timer(
      4s, std::bind(&ImuSimulatorNode::switch_pose, this));

    RCLCPP_INFO(this->get_logger(), "IMU Simulator started. Publishing to /imu/data");
  }

private:
  void publish_data()
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    // Get current ideal acceleration
    const auto & pose = poses_[current_pose_index_];

    // Add noise
    msg.linear_acceleration.x = pose[0] + accel_noise_dist_(generator_);
    msg.linear_acceleration.y = pose[1] + accel_noise_dist_(generator_);
    msg.linear_acceleration.z = pose[2] + accel_noise_dist_(generator_);

    // Gyroscope data (stationary, so ideal is 0)
    msg.angular_velocity.x = gyro_noise_dist_(generator_);
    msg.angular_velocity.y = gyro_noise_dist_(generator_);
    msg.angular_velocity.z = gyro_noise_dist_(generator_);

    // Orientation (not accurately simulated for all poses here,
    // but usually only raw accel/gyro is used for calibration)
    msg.orientation.w = 1.0;
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;

    // Covariance matrices
    msg.linear_acceleration_covariance[0] = 0.01;
    msg.linear_acceleration_covariance[4] = 0.01;
    msg.linear_acceleration_covariance[8] = 0.01;

    msg.angular_velocity_covariance[0] = 0.0001;
    msg.angular_velocity_covariance[4] = 0.0001;
    msg.angular_velocity_covariance[8] = 0.0001;

    publisher_->publish(msg);
  }

  void switch_pose()
  {
    current_pose_index_ = (current_pose_index_ + 1) % poses_.size();
    RCLCPP_INFO(this->get_logger(), "Switched to pose index: %zu", current_pose_index_);
  }

  std::vector<std::array<double, 3>> poses_;
  size_t current_pose_index_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr pose_switch_timer_;

  std::default_random_engine generator_;
  std::normal_distribution<double> accel_noise_dist_;
  std::normal_distribution<double> gyro_noise_dist_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
