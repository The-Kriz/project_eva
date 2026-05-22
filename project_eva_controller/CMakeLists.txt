cmake_minimum_required(VERSION 3.8)
project(project_eva_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
