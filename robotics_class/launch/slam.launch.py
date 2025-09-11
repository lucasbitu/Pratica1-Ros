# # Importações necessárias, incluindo as para argumentos de lançamento
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     # --- Declaração de Argumentos de Lançamento ---

#     # 1. Argumento para o arquivo de parâmetros do SLAM
#     # Isso permite que você mude o arquivo de parâmetros facilmente via linha de comando
#     declare_slam_params_file_cmd = DeclareLaunchArgument(
#         'slam_params_file',
#         default_value=os.path.join(get_package_share_directory('robotics_class'), 'params', 'default.yaml'),
#         description='Caminho completo para o arquivo de parâmetros do SLAM.'
#     )

#     # 2. Argumento para usar o tempo da simulação
#     # Essencial para garantir a sincronia com o Gazebo
#     declare_use_sim_time_cmd = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='true',
#         description='Use o tempo da simulação (Gazebo) se for true.'
#     )

#     # --- Configuração do Nó do SLAM Toolbox ---

#     # Uso do LaunchConfiguration para obter os valores dos argumentos declarados acima
#     slam_params_file = LaunchConfiguration('slam_params_file')
#     use_sim_time = LaunchConfiguration('use_sim_time')

#     # Lançando o nó do SLAM Toolbox (versão assíncrona recomendada)
#     slam_toolbox_node = Node(
#         package='slam_toolbox',
#         executable='async_slam_toolbox_node', # Mudado para async para melhor performance
#         name='slam_toolbox',
#         output='screen',
#         parameters=[
#             slam_params_file,  # Carrega os parâmetros do arquivo
#             {'use_sim_time': use_sim_time}  # Aplica o parâmetro use_sim_time
#         ],
#         # Apenas os remapeamentos que realmente mudam um tópico são necessários
#         remappings=[
#             ('/scan', '/jetauto/lidar/scan')
#         ]
#     )

#     # --- Criação e Retorno da Descrição do Lançamento ---
    
#     # A descrição de lançamento deve incluir as declarações de argumentos e o nó
#     return LaunchDescription([
#         declare_slam_params_file_cmd,
#         declare_use_sim_time_cmd,
#         slam_toolbox_node
#     ])

# Importações necessárias
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # --- Agrupamento das Declarações de Argumentos ---
    
    launch_arguments = [
        # 1. Argumento para o arquivo de parâmetros do SLAM
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(get_package_share_directory('robotics_class'), 'params', 'default.yaml'),
            description='Caminho completo para o arquivo de parâmetros do SLAM.'
        ),

        # 2. Argumento para usar o tempo da simulação
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use o tempo da simulação (Gazebo) se for true.'
        )
    ]

    # --- Agrupamento dos Nós ---

    nodes = [
        # Lançando o nó do SLAM Toolbox (versão assíncrona recomendada)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                # Uso direto do LaunchConfiguration para maior compacidade
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('/scan', '/jetauto/lidar/scan')
            ]
        )
    ]

    # --- Criação da Descrição Final do Lançamento ---
    
    # A descrição final é a soma das listas de argumentos e nós
    return LaunchDescription(launch_arguments + nodes)