from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os
# from launch.actions import ExecuteProcess


# def generate_launch_description():
#     urdf = os.path.join(get_package_share_directory('riggu_nav2'), 'urdf', 'riggu_urdf.urdf')
#     # world_file_name = 'empty.world'

#     # world = os.path.join(get_package_share_directory("riggu_nav2"), 'worlds', world_file_name)


#     return LaunchDescription([
#         #  ExecuteProcess(
#         #     cmd=['gazebo', '--verbose', world,
#         #          '-s', 'libgazebo_ros_factory.so'],
#         #     output='screen'),
#         Node(
#             package='gazebo_ros', 
#             executable='spawn_entity.py',
#             arguments=['-entity', 'robot', '-file', urdf],
#             output='screen'
#         )
#     ])
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('riggu_nav2'), 'urdf', 'riggu_urdf1.urdf')
    default_rviz_config_path=os.path.join(get_package_share_directory('riggu_nav2'), 'rviz/urdf_config.rviz')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', urdf,
                 '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        Node(
            package='gazebo_ros', 
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-file', urdf],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', default_rviz_config_path]
        ),
    ])