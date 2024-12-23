from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get the path to the URDF file
    urdf_file_path = os.path.join(
        get_package_share_directory('riggu_nav2'),
        'urdf',
        'riggu_simulation.urdf'
    )

    # Get the path to the house world
    house_world_path = os.path.join(
        '/usr/share/gazebo-11/worlds',  # Adjust based on your Gazebo version
        'house.world'
    )

    return LaunchDescription([
        # Start Gazebo and load the house world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', house_world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # Spawn the robot into Gazebo
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'Riggu', '-file', urdf_file_path, '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'),

        # Optionally start a teleop node to control the robot (keyboard control)
        Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_node',
            output='screen',
            remappings=[('/cmd_vel', '/cmd_vel')]),
    ])
