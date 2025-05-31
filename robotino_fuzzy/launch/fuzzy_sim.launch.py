from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robotino_fuzzy',
            executable='fuzzy_controller_node',
            name='fuzzy_controller',
            output='screen',
            parameters=[]
        ),
        Node(
            package='robotino_fuzzy',
            executable='sim_node.py',
            name='robotino4_simulator',
            output='screen',
            parameters=[]
        )
    ])