from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('riggu_nav2'), 'urdf', 'riggu_urdf.urdf')

    return LaunchDescription([
        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-file', urdf],
            output='screen'
        )
    ])