from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
        }]),
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 3,
                'scale_linear': 1.0,
                'scale_angular': 1.0
    }]),
        Node(
            package='robot_control',
            executable='joy_to_twist_node',
            name='joy_to_twist',
            remappings=[('/cmd_vel', '/turtle1/cmd_vel')],
        )

        
    ])
