import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_eva_description_dir = get_package_share_directory('project_eva')
    project_eva_description_share = os.path.join(get_package_prefix('project_eva'), 'share')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    # use_sim_time = LaunchConfiguration('use_sim_time')

    model_arg = DeclareLaunchArgument(name='model', default_value=os.path.join(
                                        project_eva_description_dir, 'description', 'robot.urdf.xacro'
                                        ),
                                      description='Absolute path to robot urdf file'
    )

    env_var = SetEnvironmentVariable('GAZEBO_MODEL_PATH', project_eva_description_share)

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        )
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        )
    )

    gazebo_params_file = os.path.join(project_eva_description_dir,'config','gazebo_params.yaml')

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    gazebo_ros_dir, 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
    
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')


    controller = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('project_eva_controller'),'launch','controller.launch.py'
                )])
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': project_eva_description_dir, ' use_sim_time': True}] #my addition
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        # arguments=["--frame-id", "world", "--child-frame-id", "base_link"],  #my addition
        parameters=[{'robot_description': project_eva_description_dir, ' use_sim_time': True}]
    )

    robot_localization_params_file = os.path.join(project_eva_description_dir,'config','ekf.yaml')

    robot_localization = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[robot_localization_params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
)



    return LaunchDescription([
        env_var,
        model_arg,
        # start_gazebo_server,
        # start_gazebo_client,
        gazebo,
        spawn_entity,
        static_tf,
        # node_joint_state_publisher,
        robot_state_publisher_node,
        controller,
        # robot_localization,

    ])
