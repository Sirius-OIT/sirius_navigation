import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    point_file_arg = DeclareLaunchArgument(
        'point_file', default_value='example_point.yaml'
    )

    map_dir = get_package_share_directory('sirius_navigation')
    map_path = os.path.join(map_dir, 'map', 'UMEDA_OIT_15F.yaml')

    map_server_pkg = get_package_share_directory('map_server')
    map_server_path = os.path.join(map_server_pkg, 'launch')

    config_pkg = get_package_share_directory('sirius_navigation')
    config_path = os.path.join(config_pkg, 'config', point_file_arg)

    hermite_path_planner_pkg = get_package_share_directory('hermite_path_planner_bringup')
    hermite_path_planner_path = os.path.join(hermite_path_planner_pkg, 'launch')

    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    nav2_bringup_path = os.path.join(nav2_bringup_pkg, 'launch')


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [hermite_path_planner_path, '/bringup.launch.py']
            )
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_server_config_path}],
            remappings=remappings),

        # Node(
        #     package = 'sirius_navigation',
        #     namespace = 'sirius',
        #     executable = 'move_goal',
        #     name = 'sirius_navigation',
        #     parameters = [{goal_point_file_:config_path, goal_tolerance_:2.0}]
        # ),
    ])
