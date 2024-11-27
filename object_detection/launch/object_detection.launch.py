from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection_node',
            output='screen',
            parameters=[
                #{'use_sim_time': True},
            ],
        )
    ])
