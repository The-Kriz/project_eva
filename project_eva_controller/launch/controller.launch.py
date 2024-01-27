import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

package_name='project_eva_controller'

def generate_launch_description():

    # robot_description = ParameterValue(
    #     Command(
    #         [
    #             "xacro ",
    #             os.path.join(
    #                 get_package_share_directory("project_eva"),
    #                 "description",
    #                 "robot.urdf.xacro",
    #             ),
    #         ]
    #     ),
    #     value_type=str,
    # )

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     parameters=[{"robot_description": robot_description}],
    # )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", "--controller-manager", "/controller_manager"],
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
    )

    left_finger_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_finger_controller", "--controller-manager", "/controller_manager"],
    )

    right_finger_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_finger_controller", "--controller-manager", "/controller_manager"],
    )

    head_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["head_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            # robot_state_publisher_node,
            joint_state_broadcaster_spawner,
            drive_controller_spawner,
            left_arm_controller_spawner,
            right_arm_controller_spawner,
            left_finger_controller_spawner,
            right_finger_controller_spawner,
            head_controller_spawner,
        ]
    )




