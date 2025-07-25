import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file_name",
                default_value="franka.yml",
                description="Name of the planner config file",
            ),
            Node(
                package="motion_planner",
                executable="motion_planner",
                name="cuRobo_server",
                parameters=[
                    ParameterFile(
                        os.path.join(
                            get_package_share_directory("motion_planner"),
                            "config",
                            "param.yaml",
                        )
                    ),
                    {"config_file_name": LaunchConfiguration("config_file_name")},
                ],
                output="screen",
            ),
        ]
    )
