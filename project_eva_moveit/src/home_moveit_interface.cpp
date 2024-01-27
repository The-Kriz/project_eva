#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{

    auto move_left_arm_group = moveit::planning_interface::MoveGroupInterface(node, "left_arm");
    auto move_right_arm_group = moveit::planning_interface::MoveGroupInterface(node, "right_arm");

//                                       arm   sholder  biceps  elbow   wrist   hand
    std::vector<double> left_arm_goal   {0.0,    0.08,  0.0,    0.0,    0.0,    0.0};
    std::vector<double> right_arm_goal  {0.0,   -0.08,  0.0,    0.0,    0.0,    0.0};

    bool left_arm_within_bounds = move_left_arm_group.setJointValueTarget(left_arm_goal);
    bool right_arm_within_bounds = move_right_arm_group.setJointValueTarget(right_arm_goal);

    if (!left_arm_within_bounds | !right_arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        if (!left_arm_within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
            "Left Arm Target joint position(s) were outside of limits");
        }
        if (!right_arm_within_bounds)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
            "Right Arm Target joint position(s) were outside of limits");
        }
        return;
    }



    moveit::planning_interface::MoveGroupInterface::Plan left_arm_plan;
    moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;
    bool left_arm_plan_success = (move_left_arm_group.plan(left_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool right_arm_plan_success = (move_right_arm_group.plan(right_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!left_arm_plan_success | !right_arm_plan_success)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
        if (!left_arm_plan_success)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
            "Left Arm Target joint position(s) were outside of limits");
        }
        if (!right_arm_plan_success)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
            "Right Arm Target joint position(s) were outside of limits");
        }
        return;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planner SUCCEED, moving the left and right arm");
        move_left_arm_group.move();
        move_right_arm_group.move();
    }
    

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("home_moveit_interface");
  move_robot(node);
  
//   rclcpp::spin(node);
  rclcpp::shutdown();
}