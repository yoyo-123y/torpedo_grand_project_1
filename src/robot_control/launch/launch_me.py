from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            description='Choose between sim, man, or auto'
        ),

        # SIMULATOR MODE
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen',
            condition=IfCondition(PythonExpression([mode, " == 'sim'"]))
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }],
            output='screen',
            condition=IfCondition(PythonExpression([mode, " in ['sim', 'man']"]))
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            parameters=[{
                'axis_linear.x': 1,
                'axis_angular.yaw': 3,
                'scale_linear': 1.0,
                'scale_angular': 1.0
            }],
            output='screen',
            condition=IfCondition(PythonExpression([mode, " == 'sim'"]))
        ),
        Node(
            package='robot_control',
            executable='joy_to_twist_node',
            name='joy_to_twist',
            remappings=[('/cmd_vel', '/turtle1/cmd_vel')],
            output='screen',
            condition=IfCondition(PythonExpression([mode, " in ['sim', 'man']"]))
        ),

        # REAL ROVER CONNECTION
        Node(
            package='robot_control',
            executable='esp_connect_node',
            name='esp_connect',
            output='screen',
            condition=IfCondition(PythonExpression([mode, " in ['man', 'auto']"]))
        ),

        # AUTONOMOUS MODE
        Node(
            package='robot_control',
            executable='auto_move_node',
            name='auto_move',
            output='screen',
            condition=IfCondition(PythonExpression([mode, " == 'auto'"]))
        )
    ])
