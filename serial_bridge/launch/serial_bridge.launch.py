from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='serial_bridge',
        #     executable='serial_read',
        #     name='serial_read',
        #     output='screen'
        # ),
        DeclareLaunchArgument(
            'motor_port',
            default_value='/dev/motor',
            description='The serial port to use for the motor control.'
        ),
        Node(
            package='serial_bridge',
            executable='serial_write',
            name='serial_write',
            output='screen',
            parameters=[
                {'serial_port': LaunchConfiguration('motor_port')},
            ],
        ),
    ])
