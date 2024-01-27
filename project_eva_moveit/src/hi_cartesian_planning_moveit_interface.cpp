#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("right_arm");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("hi_cartesian_planning_moveit_interface", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "right_arm";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);


  std::vector<double> right_arm_goal {-0.684, -0.080,  0.000, -2.030,  1.451, -0.390};

  // std::vector<double> right_arm_goal {-0.616, -0.330,  1.443, -1.961,  0.433, -1.233};

  move_group_arm.setJointValueTarget(right_arm_goal);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

    
  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -0.001105;
  target_pose1.orientation.y = -0.584987;
  target_pose1.orientation.z =  1.098223;
  target_pose1.orientation.w = -0.516960;
  target_pose1.position.x =     0.001093;
  target_pose1.position.y =    -0.000270;
  target_pose1.position.z =     0.856009;
  move_group_arm.setPoseTarget(target_pose1);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to Hi!");

  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose1.orientation.x -= 0.001;
  approach_waypoints.push_back(target_pose1);

  target_pose1.orientation.x -= 0.001;
  approach_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Retreat

  RCLCPP_INFO(LOGGER, "Retreat from object!");

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.orientation.x += 0.001;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.orientation.x += 0.001;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.orientation.x += 0.001;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.orientation.x += 0.001;
  retreat_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_retreat;


  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

  move_group_arm.execute(trajectory_retreat);


  std::vector<geometry_msgs::msg::Pose> way_back_waypoints;
  target_pose1.orientation.x -= 0.001;
  way_back_waypoints.push_back(target_pose1);

  target_pose1.orientation.x -= 0.001;
  way_back_waypoints.push_back(target_pose1);

  moveit_msgs::msg::RobotTrajectory trajectory_way_back;

  fraction = move_group_arm.computeCartesianPath(
      way_back_waypoints, eef_step, jump_threshold, trajectory_way_back);

  move_group_arm.execute(trajectory_approach);


  rclcpp::shutdown();
  return 0;
}