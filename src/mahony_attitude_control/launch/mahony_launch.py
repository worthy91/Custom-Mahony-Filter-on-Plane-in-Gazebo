from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Start PX4 SITL with Gazebo
        ExecuteProcess(
            cmd=['make', 'px4_sitl', 'gz_rc_cessna'],
            cwd=os.path.expanduser('~/PX4-Autopilot'),  # Adjust path as needed
            output='screen'
        ),
        
        # Start MAVROS
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[{
                'fcu_url': 'udp://:14540@127.0.0.1:14557',
                'gcs_url': '',
                'system_id': 1,
                'component_id': 1,
                'target_system_id': 1,
                'target_component_id': 1,
            }],
            output='screen'
        ),
        
        # Start Mahony controller
        Node(
            package='mahony_attitude_control',
            executable='launchq',
            name='mahony_attitude_controller',
            output='screen'
        ),
    ])
