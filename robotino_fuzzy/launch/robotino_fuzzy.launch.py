from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Robotino Odometry Node
        Node(
            package='robotino_fuzzy',
            executable='robotino_tcp_bridge.py',
            name='robotino_tcp_bridge',
            output='screen',
            parameters=[
                {'robot_x_init': 0.25},
                {'robot_y_init': 1.0}
            ]
        ),
        Node(
            package='robotino_fuzzy',
            executable='fuzzy_controller_node',
            name='fuzzy_controller',
            output='screen',
            parameters=[]
        ),
        Node(
            package='robotino_fuzzy',
            executable='robotino_viz.py',
            name='robotino4_viz',
            output='screen',
            parameters=[]
        ),
    ])