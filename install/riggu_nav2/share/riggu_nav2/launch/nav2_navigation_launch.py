
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map server node (loads the map)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/rahul/map/my_map.yaml'}]  # Update path to your map
        ),
        
        # AMCL node for localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # Lifecycle manager to manage navigation stack
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server', 'recoveries_server']}]
        ),
        
        # Planner node (to plan the path)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # Controller node (to control the movement)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        Node
        (
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            output='screen',
            # parameters=[recovery_yaml],
            parameters=[{'use_sim_time': False}]
        )
            
    ])
