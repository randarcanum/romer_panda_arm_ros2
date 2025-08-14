cmake_minimum_required(VERSION 3.5)
project(romer_publishers)

# Options
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)

# Required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(Franka REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(romer_interfaces REQUIRED)

# Executables
add_executable(cartesian_to_joint_service src/cartesian_to_joint_service.cpp)
add_executable(joint_to_cartesian_service src/joint_to_cartesian_service.cpp)
add_executable(coriolis_service src/coriolis_service.cpp)
add_executable(joint_position_publisher src/joint_position_publisher.cpp)
add_executable(ee_test_publisher src/ee_test_publisher.cpp)
add_executable(camera_test_publisher src/camera_test_publisher.cpp)

# Link dependencies
ament_target_dependencies(cartesian_to_joint_service
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  urdf
  romer_interfaces
)

ament_target_dependencies(joint_to_cartesian_service
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  urdf
  romer_interfaces
)

ament_target_dependencies(coriolis_service
  rclcpp
  sensor_msgs
  kdl_parser
  urdf
  romer_interfaces
)

ament_target_dependencies(joint_position_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  kdl_parser
  urdf
)

ament_target_dependencies(ee_test_publisher
  rclcpp
  geometry_msgs
)

ament_target_dependencies(camera_test_publisher
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS
  cartesian_to_joint_service
  joint_to_cartesian_service
  coriolis_service
  joint_position_publisher
  ee_test_publisher
  camera_test_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Tests (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()