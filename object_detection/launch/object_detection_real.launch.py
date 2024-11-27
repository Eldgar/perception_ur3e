from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detection',
            executable='object_detection_real',
            name='object_detection_real_node',
            output='screen',
            parameters=[
                #{'use_sim_time': True},
            ],
        ),
    ])
