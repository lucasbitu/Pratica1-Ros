# Importações necessárias dos pacotes launch e ament
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Gera a descrição do launch para iniciar o stack de navegação do Nav2.
    """
    # --- Definição de Caminhos e Parâmetros ---

    # Localiza o diretório do pacote 'robotics_class'
    robotics_class_pkg_dir = get_package_share_directory('robotics_class')

    # Define o caminho para o arquivo de parâmetros de navegação
    nav2_params_file = os.path.join(robotics_class_pkg_dir, 'params', 'default.yaml')

    # Define o caminho para o arquivo XML da Árvore de Comportamentos (Behavior Tree)
    behavior_tree_xml_path = os.path.join(robotics_class_pkg_dir, 'behavior_trees', 'my_nav_to_pose_bt.xml')

    # --- Declaração de Argumentos do Launch ---

    # Declara o argumento 'use_sim_time' para sincronizar com o relógio da simulação
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar o tempo de simulação (Gazebo) se for verdadeiro'
    )
    # Cria uma variável de configuração para usar o valor do argumento
    use_sim_time_config = LaunchConfiguration('use_sim_time')

    # --- Definição dos Nós do Nav2 ---

    # Nó para o Controller Server (Controlador de Movimento)
    start_controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time_config}],
        remappings=[('/cmd_vel', 'jetauto/cmd_vel')] # Remapeia o tópico de velocidade
    )

    # Nó para o Planner Server (Planejador de Caminho)
    start_planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time_config}]
    )

    # Nó para o Behavior Server (Gerenciador de Comportamentos)
    start_behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time_config}]
    )

    # Nó para o BT Navigator (Navegador baseado em Behavior Tree)
    start_bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            nav2_params_file,
            {
                'use_sim_time': use_sim_time_config,
                'default_bt_xml_filename': behavior_tree_xml_path
            }
        ],
        remappings=[('/navigate_to_pose', 'navigate_to_pose')]
    )

    # --- Gerenciamento do Ciclo de Vida (Lifecycle Manager) ---

    # Lista dos nós que serão gerenciados pelo Lifecycle Manager
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator'
    ]

    # Nó do Lifecycle Manager para iniciar e gerenciar os outros nós do Nav2
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time_config,
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )

    # Adiciona um atraso para iniciar o Lifecycle Manager
    # Isso garante que os outros nós tenham tempo de inicializar antes de serem ativados.
    delayed_lifecycle_manager_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_planner_server,
            on_start=[
                TimerAction(
                    period=2.5,  # Atraso de 2.5 segundos
                    actions=[lifecycle_manager]
                )
            ]
        )
    )

    # --- Montagem da Descrição do Launch ---

    # Retorna o objeto LaunchDescription com todos os componentes a serem executados
    return LaunchDescription([
        use_sim_time_arg,
        start_controller_server,
        start_planner_server,
        start_behavior_server,
        start_bt_navigator,
        delayed_lifecycle_manager_start
    ])