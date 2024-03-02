import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():

    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_locate').find('robot_locate')
    # Include other launch files

    # Use the launch file when use_sim_time is false
    pointcloud_to_laserscan = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('pointcloud_to_laserscan'), 'launch'), '/sample_pointcloud_to_laserscan_launch.py'
        ]),
        condition = UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    rslidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rslidar_sdk'), 'launch'), '/start.py'
        ]),
        condition = UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('miiboo_imu'), 'launch'), '/imu.launch.py'
        ]),
        condition = UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    diff_drive_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('diff_control'), 'launch'), '/diffbot.launch.py'
        ]),
        condition = UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    # this and the diff_drive_controller only one of them will be executed
    diff_drive_sim_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('diff_control'), 'launch'), '/diffbot_sim.launch.py'
        ]),
        condition = IfCondition(LaunchConfiguration('use_sim_time'))
    )

    serial_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('serial_bridge'), 'launch'), '/serial_bridge.launch.py'
        ]),
        condition = UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    twist_mux_params = os.path.join(pkg_share, 'config/twist_mux.yaml')
    twist_mux = launch_ros.actions.Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diffbot_base_controller/cmd_vel_unstamped')],
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Enable use_sim_time => use gazebo to simulate; disable use_sim_time => use real hardware'
        ),
        robot_localization_node,
        pointcloud_to_laserscan,
        rslidar_driver,
        imu_driver,
        diff_drive_controller,
        twist_mux,
        # diff_drive_sim_controller,
        serial_bridge
    ])


     