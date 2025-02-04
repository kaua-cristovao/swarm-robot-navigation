import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarm-robot-navigation').find('swarm-robot-navigation')
    default_model_path = os.path.join(pkg_share, 'src/description/rl-bot-description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'world/cenario.sdf')
    namespaceee = 'xd'

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
        arguments=['-entity', 'complete_bot', '-topic', 'robot_description'],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       namespace = namespaceee,
       parameters=[ os.path.join(pkg_share, 'config/ekf.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'odom0': f'/{namespaceee}/odom'}, 
                    {'odom0_config': [True,  True,  True,
                                        False, False, False,
                                        False, False, False,
                                        False, False, True,
                                        False, False, False]},
                    {'imu0': f'/{namespaceee}/imu'},
                    {'imu0_config': [False, False, False,
                      True,  True,  True,
                      False, False, False,
                      False, False, False,
                      False, False, False]},
                    {'base_link_frame':f'{namespaceee}_base_link'}]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'), 
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        spawn_entity,   
        robot_state_publisher_node, 
        robot_localization_node,
        rviz_node
    ])