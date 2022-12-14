import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    rviz_file_name = 'check_joint.rviz'
    rviz_path = os.path.join(
        get_package_share_directory('conix_demo'),
        'go1_description/launch',
        rviz_file_name)

    urdf_file_name = 'go1.urdf'
    urdf = os.path.join(
        get_package_share_directory('conix_demo'),
        'go1_description/urdf',
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='conix_demo',
            executable="state_to_joint_converter",
            name="state_to_joint_converter",
            output="screen",
        ),
        Node(
            package = "conix_demo",
            executable= "ros2_udp",
            name = "ros2_udp",
            output= "screen",
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', [rviz_path]]
        ),
        # Node(
        #     package='conix_demo',
        #     executable="cmd_interface",
        #     name="cmd_interface",
        #     output="screen",
        # ),
    ])