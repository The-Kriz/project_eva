#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{

    auto move_right_arm_group = moveit::planning_interface::MoveGroupInterface(node, "right_arm");

//                                        arm   sholder  biceps  elbow   wrist   hand
    std::vector<double> right_arm_goal {-0.616, -0.330,  1.443, -1.961,  0.433, -1.233};

    bool right_arm_within_bounds = move_right_arm_group.setJointValueTarget(right_arm_goal);

    if (!right_arm_within_bounds)
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                    "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
        "Right Arm Target joint position(s) were outside of limits");

        return;
    }



    moveit::planning_interface::MoveGroupInterface::Plan right_arm_plan;
    bool right_arm_plan_success = (move_right_arm_group.plan(right_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(right_arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planner SUCCEED, moving the arme and the gripper");
        move_right_arm_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "right planners failed!");
        return;
    }

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("right_namaste_moveit_interface");
  move_robot(node);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
}