import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtles_with_carrot'), 'launch'),
            '/turtle_tf2_demo.launch.py']),
        )

    return LaunchDescription([
        demo_nodes,
        
        DeclareLaunchArgument(
            'radius', default_value='3.0',
            description='Carrot translation radius'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation', default_value='1',
            description='Direction of rotation 1 ccw, -1 cw'
        ),
        Node(
            package='turtles_with_carrot',
            executable='fixed_frame_tf2_broadcaster',
            name='fixed_broadcaster',
            parameters=[
                {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')},
                {'radius': LaunchConfiguration('radius')}
            ]
        ),
    ])