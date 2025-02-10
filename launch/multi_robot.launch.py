import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import launch_ros
import os

def create_rviz_file(insert_text):
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarm-robot-navigation').find('swarm-robot-navigation')
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
    
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarm-robot-navigation').find('swarm-robot-navigation')
    num_robots = int(LaunchConfiguration('num_robots').perform(context))
    
    robot_nodes = []
    insert = ''
    for i in range(num_robots):

        robot_type = ''
        model_path = ''
        if i == 0:
            robot_type = 'Complete'
            model_path = os.path.join(pkg_share, 'src/description/complete-bot-description.urdf')
        else:
            robot_type = 'RL'
            model_path = os.path.join(pkg_share, 'src/description/rl-bot-description.urdf')
        
        robot_name = f'{robot_type}_robot{i}' 

        insert = f'''    - Alpha: 1\n      Class: rviz_default_plugins/RobotModel\n      Description Topic:\n        Depth: 5\n        Durability Policy: Volatile\n        History Policy: Keep Last\n        Reliability Policy: Reliable\n        Value: {robot_name}/robot_description\n      Enabled: true\n      Name: {robot_name}_Model\n      Visual Enabled: true\n{insert}'''
        
        # Nó para publicar o estado do robô
        robot_state_publisher = launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            parameters=[{'robot_description': Command(['xacro ', model_path,' namespace:=', robot_name])}]
        )

        # Nó para publicação do estado das juntas
        joint_state_publisher = launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=robot_name,
            arguments=[model_path]
        )

        # Nó para spawnar o robô no Gazebo
        spawn_entity = launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', robot_name, '-topic', f'{robot_name}/robot_description', '-x', str(i * 2), '-y', '0', '-z', '0.1'],
            output='screen'
        )

        # Adiciona os nós gerados à lista
        robot_nodes.extend([robot_state_publisher, joint_state_publisher, spawn_entity])
    create_rviz_file(insert)
    return robot_nodes

def generate_launch_description():
    """Cria a estrutura do launch file"""

    # Diretório base do pacote
    pkg_share = launch_ros.substitutions.FindPackageShare(package='swarm-robot-navigation').find('swarm-robot-navigation')

    # Caminhos de arquivos globais
    dynamic_rviz_config_path = os.path.join(pkg_share, 'rviz/dynamic_urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'world/cenario.sdf')

    # Nó do RViz (compartilhado entre todos os robôs)
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        # Argumentos do launch
        launch.actions.DeclareLaunchArgument('num_robots', default_value='4', description='Número de robôs na simulação'),
        launch.actions.DeclareLaunchArgument('rvizconfig', default_value=dynamic_rviz_config_path, description='Configuração do RViz'),
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='True', description='Ativar tempo de simulação'),

        # Iniciar o Gazebo com o cenário
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path],
            output='screen'
        ),

        # Adicionar os nós dinâmicos gerados
        OpaqueFunction(function=generate_robots),

        # Nó do RViz
        rviz_node
    ])

