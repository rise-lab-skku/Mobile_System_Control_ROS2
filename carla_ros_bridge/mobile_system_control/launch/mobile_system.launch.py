from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobile_system_control',
            executable='mobile_system_control',
            name='mobile_system_control',
            output='screen',
            emulate_tty=True, # To see RCLCPP_INFO messages
            parameters=[
                # Add any parameters here if needed in the future
            ],
            remappings=[
                # Remap topics similarly to the ROS1 launch file
                # ROS2 uses node_name/topic_name for private topics by default
                ('state2user', '/mobile_system_control/ego_vehicle'),
                ('ctrl2carla', '/carla/ego_vehicle/vehicle_control_cmd'),
                ('carla_ego', '/carla/ego_vehicle/vehicle_status'),
                ('carla_obj', '/carla/ego_vehicle/odometry'),
                ('user_ctrl', '/mobile_system_control/control_msg'),
            ]
        )
    ])