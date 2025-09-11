# import os
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

# def generate_launch_description():
#     pkg_robotics_class = get_package_share_directory('robotics_class')

#     use_sim_time = DeclareLaunchArgument(
#         'use_sim_time', default_value='true',
#         description='Use simulation/Gazebo clock')

#     map_server = Node(
#         package='nav2_map_server',
#         executable='map_server',
#         name='map_server',
#         output='screen',
#         parameters=[os.path.join(pkg_robotics_class, 'params', 'default.yaml')])

#     amcl = Node(
#         package='nav2_amcl',
#         executable='amcl',
#         name='amcl',
#         output='screen',
#         parameters=[os.path.join(pkg_robotics_class, 'params', 'default.yaml')])
    
#     lifecycle_manager = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_localization',
#         output='screen',
#         parameters=[{
#             'autostart': True,
#             'node_names': ['map_server', 'amcl']
#         }])
    
#     return LaunchDescription([
#         use_sim_time,
#         map_server,
#         amcl,
#         lifecycle_manager
#     ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Caminho para o diretório de compartilhamento do pacote
    pkg_robotics_class = get_package_share_directory('robotics_class')
    
    # Define o caminho para o arquivo de parâmetros uma única vez
    params_file = os.path.join(pkg_robotics_class, 'params', 'default.yaml')
    
    # Agrupa a declaração de todos os argumentos de lançamento
    launch_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_robotics_class, 'maps', 'class_map.yaml'),
            description='Full path to map file to load'),
            
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Full path to the ROS2 parameters file to use')
    ]

    # Agrupa todos os nós a serem lançados
    nodes = [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'yaml_filename': LaunchConfiguration('map')}
            ]),
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ])
    ]

    # Retorna a descrição de lançamento combinando os argumentos e os nós
    return LaunchDescription(launch_arguments + nodes)