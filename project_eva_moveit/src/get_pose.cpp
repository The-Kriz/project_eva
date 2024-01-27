#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Declare Node
  std::shared_ptr<rclcpp::Node> node =
    std::make_shared<rclcpp::Node>("get_pose",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(
        true));

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
    "Right Arm current pose of the end effector");

  // Create the MoveIt MoveGroup Interface
  moveit::planning_interface::MoveGroupInterface right_arm_move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "right_arm");

  // print current pose
  geometry_msgs::msg::Pose right_arm_current_pose =
    right_arm_move_group_interface.getCurrentPose().pose;

  // Print the current pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    right_arm_current_pose.position.x,
    right_arm_current_pose.position.y,
    right_arm_current_pose.position.z,
    right_arm_current_pose.orientation.x,
    right_arm_current_pose.orientation.y,
    right_arm_current_pose.orientation.z,
    right_arm_current_pose.orientation.w);

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
    "Left Arm current pose of the end effector");

  moveit::planning_interface::MoveGroupInterface left_arm_move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "left_arm");

  // print current pose
  geometry_msgs::msg::Pose left_arm_current_pose =
    left_arm_move_group_interface.getCurrentPose().pose;

  // Print the current pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    left_arm_current_pose.position.x,
    left_arm_current_pose.position.y,
    left_arm_current_pose.position.z,
    left_arm_current_pose.orientation.x,
    left_arm_current_pose.orientation.y,
    left_arm_current_pose.orientation.z,
    left_arm_current_pose.orientation.w);

  RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
    "Head current pose of the end effector");

  moveit::planning_interface::MoveGroupInterface head_move_group_interface =
    moveit::planning_interface::MoveGroupInterface(node, "head");

  // print current pose
  geometry_msgs::msg::Pose head_current_pose =
    head_move_group_interface.getCurrentPose().pose;

  // Print the current pose of the end effector
  RCLCPP_INFO(node->get_logger(), "Current pose: %f %f %f %f %f %f %f",
    head_current_pose.position.x,
    head_current_pose.position.y,
    head_current_pose.position.z,
    head_current_pose.orientation.x,
    head_current_pose.orientation.y,
    head_current_pose.orientation.z,
    head_current_pose.orientation.w);

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();
  return 0;
}