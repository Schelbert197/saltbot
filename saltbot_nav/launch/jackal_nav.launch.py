from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments=[
                ('params_file',
                    PathJoinSubstitution([
                        FindPackageShare('saltbot_nav'),
                        'config',
                        'nav2_params.yaml'
                    ])
                 ),
            ],
        ),
    ])
