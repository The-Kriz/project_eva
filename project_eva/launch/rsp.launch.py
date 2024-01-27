import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_drive = LaunchConfiguration('use_drive')
    use_eva = LaunchConfiguration('use_eva')
    use_omni_wheel = LaunchConfiguration('use_omni_wheel')


    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('project_eva'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_config = Command(['xacro ', xacro_file, ' use_sim_time:=',use_sim_time,' use_omni_wheel:=',use_omni_wheel,' use_eva:=',use_eva,' use_drive:=',use_drive])
    
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, ' use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a joint_state_publisher node
    params = {'robot_description': robot_description_config, ' use_sim_time': use_sim_time}
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'use_omni_wheel',
            default_value='false',
            description='Use omni wheel if true'),
        DeclareLaunchArgument(
            'use_eva',
            default_value='true',
            description='Use arm if true'),
        DeclareLaunchArgument(
            'use_drive',
            default_value='true',
            description='Only drive if true'),

        node_robot_state_publisher
        # node_joint_state_publisher

    ])