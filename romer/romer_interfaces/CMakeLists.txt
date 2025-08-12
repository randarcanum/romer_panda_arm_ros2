cmake_minimum_required(VERSION 3.5)
project(romer_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CartesianToJoint.srv"
  "srv/JointToCartesian.srv"
  "srv/Coriolis.srv"
  DEPENDENCIES geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs sensor_msgs)
ament_package()