import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.action import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    point_file_arg = DeclareLaunchArgument(
        'point_file', default_value='example_point.yaml'
    )

    config_pkg = get_package_share_directory('sirius_navigation')
    config_path = os.path.join(config_pkg, 'config', point_file_arg)

    hermite_path_planner_pkg = get_package_share_directory('hermite_path_planner_bringup')
    hermite_path_planner_path = os.path.join(hermite_path_planner_pkg, 'launch')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [hermite_path_planner_path, '/bringup.launch.py']
            )
        ),

        Node(
            package = 'sirius_navigation',
            namespace = 'sirius',
            executable = 'sirius_navigation',
            name = 'sirius_navigation',
            parameters = [{goal_point_file_:config_path, goal_tolerance_:2.0}]
        ),
    ])
