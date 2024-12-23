from rclpy.node import Node
import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='riggu_nav2').find('riggu_nav2')
    default_model_path = os.path.join(pkg_share, 'urdf/riggu_final.urdf')
    default_scan_filter_config_path = os.path.join(pkg_share, 'config/laser_scan.yaml')

    # Robot State Publisher Node (Needed for TF broadcasting)
    robot_state_publisher_node = launch_ros.actions.Node(
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_robot_state_pub')),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        arguments=[default_model_path]
    )

    # Joint State Publisher Node (Needed for robot joint state)
    joint_state_publisher_node = launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    # Odometry Node (to publish odometry information)
    odometry = launch_ros.actions.Node(
        package="riggu_nav2",  # Make sure this is the correct package for your odometry node
        executable="bot_vel_pub.py",  # Ensure this is the correct executable for your odometry node
        name="odometry",
    )

    # RPLIDAR Launch (for laser scan data)
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'), 'launch'), '/rplidar_a1_launch.py'])
    )

    # Laser Scan Filter (used to filter scan data)
    scan_filter = launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[default_scan_filter_config_path]
    )

    # Return the complete launch description
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, 
                                             description="Absolute path to robot urdf file"),
        launch.actions.DeclareLaunchArgument(name='use_robot_state_pub', default_value='True',
                                             description='Whether to start the robot state publisher'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        odometry,  # Including the odometry node
        rplidar,
        scan_filter
    ])
