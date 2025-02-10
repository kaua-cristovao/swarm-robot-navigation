import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarm-robot-navigation').find('swarm-robot-navigation')
    default_model_path = os.path.join(pkg_share, 'src/description/rl-bot-description.urdf')
    default_nav2_config = os.path.join(pkg_share,'config/nav2_params.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/basic_urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'world/cenario.sdf')
    namespaceee = 'xd'


    slam_toolbox_pkg = launch_ros.substitutions.FindPackageShare(package='slam_toolbox').find('slam_toolbox')


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespaceee,
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model'),' namespace:=', namespaceee])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace = namespaceee,
        arguments=[default_model_path], #Add this line
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
        namespace = namespaceee,
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'complete_bot', '-topic', 'robot_description','-x', '1', '-y', '-1', '-z', '0.1'],
        output='screen'
    )

    slam_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([os.path.join(slam_toolbox_pkg, 'launch'),'/online_async_launch.py']),
         launch_arguments = {'use_sim_time': 'True','slam_params_file':default_nav2_config}.items(),
    )

    gazebo = launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen')

    return launch.LaunchDescription([
        
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='slam_params_file',default_value=default_nav2_config,
                                            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
        spawn_entity,
        gazebo,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        slam_launch
    ])