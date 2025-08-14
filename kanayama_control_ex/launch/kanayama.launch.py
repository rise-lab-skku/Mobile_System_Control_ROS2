from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory('kanayama_control_ex')
    path_csv = os.path.join(share_dir, 'track', 'reference_path.csv')

    return LaunchDescription([
        Node(
            package='kanayama_control_ex',
            executable='kanayama_control_ex_node',
            name='kanayama_control_ex_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'path': path_csv},
                {'K_x': 1.0},
                {'K_y': 0.08},
                {'K_t': 0.05}
            ]
        )
    ])