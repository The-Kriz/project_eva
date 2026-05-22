#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>


void move_robot(const std::shared_ptr<rclcpp::Node> node)
{

    auto move_left_arm_group = moveit::planning_interface::MoveGroupInterface(node, "left_arm");
    auto move_right_arm_group = moveit::planning_interface::MoveGroupInterface(node, "right_arm");
    auto move_both_arm_group = moveit::planning_interface::MoveGroupInterface(node, "both_arms");

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

    std::vector<double> left_arm_joint_value;
    std::vector<double> right_arm_joint_value;


    move_left_arm_group.getJointValueTarget(left_arm_joint_value);
    move_right_arm_group.getJointValueTarget(right_arm_joint_value);

    std::vector<double> both_arm_joints_send;

    both_arm_joints_send.insert(both_arm_joints_send.end(), left_arm_joint_value.begin(),
                          left_arm_joint_value.end());
    both_arm_joints_send.insert(both_arm_joints_send.end(), right_arm_joint_value.begin(),
                          right_arm_joint_value.end());

    auto successCheck = move_both_arm_group.setJointValueTarget(both_arm_joints_send);

    if (successCheck) {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "left and right arm joint value combined successfully.");
    } 
    else 
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "left and right arm joint value failed to combine");
        return;
    }



    std::vector<double> both_arm_joint_value;
    move_both_arm_group.getJointValueTarget(both_arm_joint_value);


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planning_success =(move_both_arm_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (planning_success) {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planning succeeded. Executing movement...");
        move_both_arm_group.execute(plan);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Movement executed successfully.");
    } 
    else 
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planning failed for both arms");
        return;
    }

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("both_home_moveit_interface");
  move_robot(node);
  
//   rclcpp::spin(node);
  rclcpp::shutdown();
}

