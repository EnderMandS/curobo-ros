import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="motion_planner",
            executable="motion_planner",
            name="cuRobo_server",
            parameters=[ParameterFile(os.path.join(
                get_package_share_directory('motion_planner'),
                'config',
                'param.yaml'
            ))],
            output="screen",
        ),
    ])
