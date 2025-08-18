import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('pid_control_ex')
    path_to_csv = os.path.join(share_dir,'track', 'reference_path.csv')

    return LaunchDescription([
        Node(
            package='pid_control_ex',
            executable='pid_control_ex_node',
            name='pid_control_ex_node',
            output='screen',
            emulate_tty=True, # ROS 2에서 screen 출력을 보기 위해 필요
            parameters=[
                {'path': path_to_csv},
                {'Kp': 2.0},
                {'Ki': 0.5},
                {'accel': 0.8}
            ]
        )
    ])