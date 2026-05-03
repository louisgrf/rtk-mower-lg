from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_path = os.path.expanduser('~/rtk-mower-lg/config/ekf.yaml')

    return LaunchDescription([
        # Static transform: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),

        # Static transform: base_link -> gps
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gps',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps'],
        ),

        # GPS Node
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='gps_node',
            parameters=[{
                'device': '/dev/ttyACM0',
                'uart1.baudrate': 38400,
                'tmode3': 0,
                'nav_rate': 1,
            }],
            output='screen',
        ),

        # IMU Node
        Node(
            package='bno085',
            executable='bno085_publisher',
            name='imu_node',
            output='screen',
        ),

        # EKF Fusion
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[config_path],
            output='screen',
        ),

        # GPS -> Odometry Transform
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[config_path],
            remappings=[
                ('imu', 'IMU_Data'),
                ('gps/fix', 'fix'),
                ('odometry/filtered', 'odometry/filtered'),
            ],
            output='screen',
        ),
    ])
