import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from os.path import join

def generate_launch_description():
    # Pacotes 
    pkg_share = FindPackageShare(package='swarm-robot-navigation').find('swarm-robot-navigation')
    slam_toolbox_pkg = FindPackageShare(package='slam_toolbox').find('slam_toolbox')
    nav2_bringup_pkg = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    

    # Arquivos 
    default_model_path = join(pkg_share, 'src/description/turtle-rl-bot-description.urdf')
    default_nav2_config = join(pkg_share,'config/nav2_params.yaml')
    default_nav2bring_config = join(pkg_share,'config/nav2bringup_params.yaml')
    default_rviz_config_path = join(pkg_share, 'rviz/basic_urdf_config.rviz')
    world_path = join(pkg_share, 'world/cenario.sdf')

    # Configs
    namespaceee = 'xd'

    lifecycle_nodes = ['controller_server','planner_server','bt_navigator']

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespaceee,
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model'),' namespace:=', namespaceee])}]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace = namespaceee,
        arguments=[default_model_path],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = Node(
        namespace = namespaceee,
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rl_bot', '-topic', 'robot_description','-x', '1', '-y', '-1', '-z', '0.2'],
        output='screen'
    )
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[default_nav2bring_config,{'robot_base_frame':'xd_base_link'}],
    )
    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        parameters=[default_nav2bring_config,{'robot_base_frame':'xd_base_link'}],
    )
    nav2_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        parameters=[default_nav2bring_config, {'default_nav_to_pose_bt_xml': join(pkg_share,'behavior_tree/basic_tree.xml'),'default_nav_through_poses_bt_xml': join(pkg_share,'behavior_tree/basic_tree.xml'),'robot_base_frame':'xd_base_link'}],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}]
    )

    # Launches secundários
    slam_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([join(slam_toolbox_pkg, 'launch'),'/online_async_launch.py']),
         launch_arguments = {'use_sim_time': 'True','slam_params_file':default_nav2_config}.items(),
    )

    nav2_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([join(nav2_bringup_pkg, 'launch'),'/navigation_launch.py']),
         launch_arguments = {'use_sim_time': 'True','params_file':default_nav2bring_config}.items(),
    )

    # Comandos
    gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen')

    return launch.LaunchDescription([
        
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='slam_params_file',default_value=default_nav2_config,
                                            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),
        
        spawn_entity,
        gazebo,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        # nav2_launch,
        slam_launch,
        nav2_controller,
        nav2_planner,
        nav2_bt_navigator,
        lifecycle_manager
    ])