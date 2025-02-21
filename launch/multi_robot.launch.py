import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def create_rviz_file(insert_text):
    pkg_share = get_package_share_directory('swarm-robot-navigation')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    dynamic_rviz_config_path = os.path.join(pkg_share, 'rviz/dynamic_urdf_config.rviz')

    with open(dynamic_rviz_config_path, 'w') as dynamic_file:
        with open(default_rviz_config_path, 'r') as urdf_file:

            lines = urdf_file.readlines()
            for line in lines:
                dynamic_file.write(line)  
                if 'Name: Grid' in line:
                    dynamic_file.write(insert_text) 
                
            
    return

def generate_robots(context):
    """Função para criar os nós dinamicamente com base em num_robots"""
    
    pkg_share = get_package_share_directory('swarm-robot-navigation')
    # bringup_dir = get_package_share_directory('nav2_bringup')
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    params_file = LaunchConfiguration('params_file')
    
    robot_namespaces = []
    robot_nodes = []
    insert = ''
    for i in range(num_robots):

        robot_type = ''
        model_path = ''
        if i == 0:
            robot_type = 'Complete'
            model_path = os.path.join(pkg_share, 'src/description/multi-complete-bot-description.urdf')
        else:
            robot_type = 'RL'
            model_path = os.path.join(pkg_share, 'src/description/multi-rl-bot-description.urdf')
        
        robot_name = f'{robot_type}_robot{i}' 

        insert = f'''    - Alpha: 1\n      Class: rviz_default_plugins/RobotModel\n      Description Topic:\n        Depth: 5\n        Durability Policy: Volatile\n        History Policy: Keep Last\n        Reliability Policy: Reliable\n        Value: {robot_name}/robot_description\n      Enabled: true\n      Name: {robot_name}_Model\n      Visual Enabled: true\n{insert}'''
        
        robot_namespaces.append(robot_name)
        
        x_init = (i % 2) * 0.5 
        y_init = (-1)**(i%2) * 1.5 
        z_init = 0.1

        # Nó para publicar o estado do robô
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{'robot_description': Command(['xacro ', model_path,' namespace:=', robot_name])}]
        )

        # Nó para publicação do estado das juntas
        joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_name,
            arguments=[model_path]
        )

        # Nó para spawnar o robô no Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', robot_name, '-topic', f'{robot_name}/robot_description', '-x', str(x_init), '-y', str(y_init), '-z', str(z_init)],
            output='screen'
        )

        # Publica a transformação estática map -> robotX/odom
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_map_to_{robot_name}_odom",
            arguments=[
                str(x_init), str(y_init), str(z_init),  # Posição inicial (x, y, z)
                "0", "0", "0",      # Orientação (roll, pitch, yaw)
                "map", f"{robot_name}/odom"     # Frames de origem e destino
            ]
        )
 
        start_sync_slam_toolbox_node = Node(
            parameters=[params_file,{
                                     'odom_frame':   TextSubstitution(text = f'{robot_name}/odom'),
                                     'base_frame':   TextSubstitution(text = f'{robot_name}/base_link'),
                                    #  'scan_topic':   TextSubstitution(text = f'{robot_name}/scan'),
                                     'use_sim_time': LaunchConfiguration('use_sim_time')}
                        ],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            namespace = robot_name, 
            output='screen',
            remappings=[('/scan', f'/{robot_name}/scan'),('/map','map')]
        )
        
        # Adiciona os nós gerados à lista
        robot_nodes.extend([robot_state_publisher, joint_state_publisher, spawn_entity, static_tf,start_sync_slam_toolbox_node])

    create_rviz_file(insert)

    # Nó do MergedMap (compartilhado entre todos os robôs)
    merged_map = Node(
        package='swarm-robot-navigation',
        executable='merged_map',
        name='merged_map',
        output='screen',
        parameters=[{
                'robot_namespaces': robot_namespaces,
                # 'publish_frequency':0.5,
            }]
    )
    robot_nodes.append(merged_map)
    return robot_nodes

def generate_launch_description():
    """Cria a estrutura do launch file"""

    # Diretório base do pacote
    pkg_share = get_package_share_directory('swarm-robot-navigation')
    bringup_dir = get_package_share_directory('nav2_bringup')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')

    # Caminhos de arquivos globais
    dynamic_rviz_config_path = os.path.join(pkg_share, 'rviz/dynamic_urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'world/cenario.sdf')

    # Nó do RViz (compartilhado entre todos os robôs)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # start_slam_toolbox_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(slam_launch_file),
    #     launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
    #                       'slam_params_file': LaunchConfiguration('params_file')}.items(),
    # )

    

    return LaunchDescription([
        # Argumentos do launch
        DeclareLaunchArgument('num_robots', default_value='2', description='Número de robôs na simulação'),
        DeclareLaunchArgument('rvizconfig', default_value=dynamic_rviz_config_path, description='Configuração do RViz'),
        DeclareLaunchArgument('use_sim_time', default_value='True', description='Ativar tempo de simulação'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'), description='Caminho para os parâmetros da simulação'),
        # Iniciar o Gazebo com o cenário
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        # Adicionar os nós dinâmicos gerados
        OpaqueFunction(function=generate_robots),

        # Nó do RViz
        rviz_node,
    ])

