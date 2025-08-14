from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory('purepursuit_control_ex')
    path_csv = os.path.join(share_dir, 'track', 'reference_path.csv')

    return LaunchDescription([
        Node(
            package='purepursuit_control_ex',
            executable='purepursuit_control_ex_node',
            name='purepursuit_control_ex_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'path': path_csv},
                {'LD': 5.0}
            ]
        )
    ])