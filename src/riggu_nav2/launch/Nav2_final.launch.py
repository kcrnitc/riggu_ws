from rclpy.node import Node
import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource 


def generate_launch_description():
    pkg_share=launch_ros.substitutions.FindPackageShare(package='riggu_nav2').find('riggu_nav2')
    default_model_path=os.path.join(pkg_share, 'urdf/riggu_final.urdf')
    #default_rviz_config_path=os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    default_scan_filter_config_path=os.path.join(pkg_share, 'config/laser_scan.yaml')
    slam_toolbox_params_config_path=os.path.join(pkg_share, 'config/mapper_params_online_async.yaml')
    

    robot_state_publisher_node=launch_ros.actions.Node(
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_robot_state_pub')),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
        arguments=[default_model_path]
    )
    
    joint_state_publisher_node=launch_ros.actions.Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}]
    )

    odometry=launch_ros.actions.Node(
        package="riggu_nav2",
        executable="bot_vel_pub.py",
        name="Odometry",
    )

    # pid=launch_ros.actions.Node(
    #     package="riggu_nav2",
    #     executable="pid_control.py",
    #     name="PID",
    # )

    # joy=launch_ros.actions.Node(
    #     package="riggu_nav2",
    #     executable="joy.py",
    #     name="JOY",
    # )

    # joy_node=launch_ros.actions.Node(
    #     package="joy",
    #     executable="joy_node",
    #     name="joy_node",
    # )

    # rviz_node=launch_ros.actions.Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=['-d', LaunchConfiguration("rviz_config")]
    # )

    scan_filter=launch_ros.actions.Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[default_scan_filter_config_path]
    )

    rplidar=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('rplidar_ros'),'launch'),'/rplidar_a1_launch.py'])
    )
    
    slam_toolbox=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam_toolbox'),'launch'),'/online_async_launch.py']),
        launch_arguments={'params_file':slam_toolbox_params_config_path,}.items(),
    )


    

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, 
                                             description="Absolute path to robot urdf file"),
        # launch.actions.DeclareLaunchArgument(name='rviz_config', default_value=default_rviz_config_path,
        #                                     description="Absolute path to rviz config file"),
        launch.actions.DeclareLaunchArgument(name='use_robot_state_pub',default_value='True',
                                             description='Whether to start the robot state publisher'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        odometry,
        #rviz_node,
        rplidar,
        scan_filter,
        slam_toolbox,
        # joy,
        # joy_node,
        # pid
    ])