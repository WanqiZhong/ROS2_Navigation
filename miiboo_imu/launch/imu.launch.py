from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/imu',
            description='The serial port to use for the IMU.'
        ),
        Node(
            package='miiboo_imu',
            executable='imu_read',
            namespace='demo',
            name='imu',
            output='screen',
            parameters=[
                {'com_port': LaunchConfiguration('imu_port')},
                {'imu_frame_id': 'imu_link'},
                {'mag_frame_id': 'imu_link'},
                {'imu_topic': '/demo/imu'},
                {'mag_topic': '/mag'},
            ],
        ),
    ])
