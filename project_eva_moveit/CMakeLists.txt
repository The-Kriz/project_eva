cmake_minimum_required(VERSION 3.8)
project(project_eva_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(simple_moveit_interface src/simple_moveit_interface.cpp)
ament_target_dependencies(simple_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(namaste_moveit_interface src/namaste_moveit_interface.cpp)
ament_target_dependencies(namaste_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(home_moveit_interface src/home_moveit_interface.cpp)
ament_target_dependencies(home_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(left_namaste_moveit_interface src/left_namaste_moveit_interface.cpp)
ament_target_dependencies(left_namaste_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(right_namaste_moveit_interface src/right_namaste_moveit_interface.cpp)
ament_target_dependencies(right_namaste_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(both_namaste_moveit_interface src/both_namaste_moveit_interface.cpp)
ament_target_dependencies(both_namaste_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(both_home_moveit_interface src/both_home_moveit_interface.cpp)
ament_target_dependencies(both_home_moveit_interface rclcpp moveit_ros_planning_interface)

add_executable(get_pose src/get_pose.cpp)
ament_target_dependencies(get_pose rclcpp moveit_ros_planning_interface)

add_executable(hi_cartesian_planning_moveit_interface src/hi_cartesian_planning_moveit_interface.cpp)
ament_target_dependencies(hi_cartesian_planning_moveit_interface rclcpp moveit_ros_planning_interface)



install(DIRECTORY 
        launch 
        config 
        src
        DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  get_pose
  both_home_moveit_interface
  both_namaste_moveit_interface
  simple_moveit_interface
  namaste_moveit_interface
  left_namaste_moveit_interface
  right_namaste_moveit_interface
  home_moveit_interface
  hi_cartesian_planning_moveit_interface
  DESTINATION lib/${PROJECT_NAME}

)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()