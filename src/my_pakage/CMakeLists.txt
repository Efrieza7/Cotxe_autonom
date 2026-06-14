cmake_minimum_required(VERSION 3.5)
project(my_pakage)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConsMap.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
