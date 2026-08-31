cmake_minimum_required(VERSION 3.10)
project(imu_svd_calib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Define custom interfaces (Actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CalibrateImu.action"
  DEPENDENCIES sensor_msgs std_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Add dummy simulation node
add_executable(imu_simulator_node src/imu_simulator_node.cpp)
target_include_directories(imu_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_simulator_node PUBLIC c_std_99 cxx_std_17)  # Require C++17
target_link_libraries(imu_simulator_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
)

# Add calibration node
add_executable(imu_calibration_node
  src/imu_calibration_node.cpp
  src/svd_calibration.cpp
  src/lm_calibration.cpp
)
target_include_directories(imu_calibration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(imu_calibration_node PUBLIC c_std_99 cxx_std_17)  # Require C++17
target_link_libraries(imu_calibration_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  sensor_msgs::sensor_msgs
  std_msgs::std_msgs
  tf2::tf2
  tf2_ros::tf2_ros
  visualization_msgs::visualization_msgs
  Eigen3::Eigen
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imu_calibration_node "${cpp_typesupport_target}")

install(TARGETS
  imu_simulator_node
  imu_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pose_window test/test_pose_window.cpp)
  target_include_directories(test_pose_window PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_pose_window Eigen3::Eigen)

  ament_add_gtest(test_calibration
    test/test_calibration.cpp
    src/svd_calibration.cpp
    src/lm_calibration.cpp
  )
  target_include_directories(test_calibration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(test_calibration Eigen3::Eigen)
endif()

ament_package()
