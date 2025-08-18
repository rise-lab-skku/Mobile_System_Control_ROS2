from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory('mpc_control_ex')
    path_csv = os.path.join(share_dir, 'track', 'reference_path.csv')

    return LaunchDescription([
        Node(
            package='mpc_control_ex',
            executable='mpc_control_ex_node',
            name='mpc_control_ex_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'path': path_csv},
                {'ControlNum': 20},
                {'Q': 3.0},
                {'P': 1.0},
                {'R': 1.0},
                {'x_threshold': 8.0},
                {'y_threshold': 8.0},
                {'head_threshold': 360.0},
                {'time_interval': 0.1},
                {'K_x': 1.0},
                {'K_y': 0.08},
                {'K_t': 0.05}
            ]
        )
    ])