from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stretch_wacc_sensor',
            executable='udp_processor',
            name='wacc_sensor_node',
            output='screen'
        )
    ])