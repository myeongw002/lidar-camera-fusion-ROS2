"""Start the processing node; sensor topics must already be available."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('lidar_camera_fusion')
    arguments = [
        DeclareLaunchArgument('pcTopic', default_value='/velodyne_points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([share, 'config', 'interpolated.yaml'])),
    ]
    overrides = {
        'pcTopic': ParameterValue(LaunchConfiguration('pcTopic'), value_type=str),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    parameters = [LaunchConfiguration('params_file')]
    parameters.append(overrides)
    return LaunchDescription(arguments + [
        Node(package='lidar_camera_fusion', executable='interpolated_node',
             name='interpolated_node', parameters=parameters, output='screen'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'interpoled.rviz'])],
             parameters=[{'use_sim_time': overrides['use_sim_time']}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
