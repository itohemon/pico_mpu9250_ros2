import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='pico_mpu9250_ros2').find('pico_mpu9250_ros2')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/mpu9250_i2c.rviz')
    
    uros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial","--dev","/dev/ttyACM0"],
        output="screen"
    )

    add_timestamp = Node(
        package='pico_mpu9250_ros2',
        executable='add_timestamp',
        name='add_timestamp',
        output="screen"
    )
        
    imu_madgwick_pkg_dir = LaunchConfiguration(
        'imu_filter_madgwick_pkg_dir',
        default=os.path.join(get_package_share_directory('imu_filter_madgwick'), 'launch')
    )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),        

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([imu_madgwick_pkg_dir, '/imu_filter.launch.py']),
            ),
        
        uros_agent_node,
        add_timestamp,
        rviz_node
        ])
