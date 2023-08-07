import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_wheel_odometry = LaunchConfiguration('use_wheel_odometry', default='false')

    urdf = './../coppeliasim_simulation/urdf_from_coppelia/differential_drive.urdf'
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            "use_wheel_odometry", 
            default_value='false',
            description="Use wheel odometry if true"),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc, 'use_wheel_odometry': use_wheel_odometry}],
            arguments=[urdf]),
        Node(
            package='state_estimation',
            executable='state_publisher',
            name='state_publisher',
            output='screen',
            parameters=[{'use_wheel_odometry': use_wheel_odometry}]),
        Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "laser_frame"])
    ])

