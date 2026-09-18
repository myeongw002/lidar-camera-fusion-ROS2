"""Start interpolation and camera fusion; sensor topics must already be available."""
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
        DeclareLaunchArgument('interpolatedTopic', default_value='/pc_interpoled'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'interpolation_params_file',
            default_value=PathJoinSubstitution([share, 'config', 'interpolated.yaml'])),
        DeclareLaunchArgument(
            'fusion_params_file',
            default_value=PathJoinSubstitution([share, 'config', 'fusion.yaml'])),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=PathJoinSubstitution([share, 'config', 'calibration.yaml'])),
    ]

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    interpolation_overrides = {
        'pcTopic': ParameterValue(LaunchConfiguration('pcTopic'), value_type=str),
        'output_cloud_topic': ParameterValue(LaunchConfiguration('interpolatedTopic'), value_type=str),
        'use_sim_time': use_sim_time,
    }
    fusion_overrides = {
        'pcTopic': ParameterValue(LaunchConfiguration('interpolatedTopic'), value_type=str),
        'rawPcTopic': ParameterValue(LaunchConfiguration('pcTopic'), value_type=str),
        'use_sim_time': use_sim_time,
    }

    return LaunchDescription(arguments + [
        Node(
            package='lidar_camera_fusion',
            executable='interpolated_node',
            name='interpolated_node',
            parameters=[LaunchConfiguration('interpolation_params_file'), interpolation_overrides],
            output='screen'),
        Node(
            package='lidar_camera_fusion',
            executable='lidar_camera_node',
            name='lidar_camera_node',
            parameters=[
                LaunchConfiguration('fusion_params_file'),
                LaunchConfiguration('calibration_file'),
                fusion_overrides,
            ],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', PathJoinSubstitution([share, 'rviz', 'lidar_camera.rviz'])],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
