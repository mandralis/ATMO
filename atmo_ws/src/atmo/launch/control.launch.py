from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='atmo',
            executable='mpc_controller_hardware',
            name='mpc_controller_hardware',
            output='screen',    
        ),
        Node(
            package='atmo',
            executable='tilt_controller_hardware',
            name='tilt_controller_hardware',
            output='log',    
        ),        
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '/mpc_status'],
            output='screen'
        )
    ])
