import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_node',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{'lookahead_distance': 1.0, 'max_speed': 0.5}],
        ),
        
        # Node(
        #     package='path_planning_package',
        #     executable='path_planning_node',
        #     name='path_planning_node',
        #     output='screen',
        # ),
    ])
