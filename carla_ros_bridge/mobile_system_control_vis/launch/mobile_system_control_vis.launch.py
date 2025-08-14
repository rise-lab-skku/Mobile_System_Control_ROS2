import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vis_pkg_path = get_package_share_directory('mobile_system_control_vis')

    visualizer_node = Node(
        package='mobile_system_control_vis',
        executable='mobile_system_control_vis',
        name='mobile_system_control_vis',
        output='screen',
        remappings=[
            # CARLA에서 직접 발행하는 Odometry 토픽을 구독하도록 수정합니다.
            ('pose', '/carla/ego_vehicle/odometry'),
            ('track', '/mobile_system_control_vis/track'),
            ('vehicle', '/mobile_system_control_vis/vehicle')
        ],
        parameters=[{
            'csv_dir': os.path.join(vis_pkg_path, 'track', 'reference_path.csv')
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(vis_pkg_path, 'rviz', 'test.rviz')]
    )

    # 'map'과 'odom' 좌표계를 연결하여 'Frame [map] does not exist' 오류를 해결합니다.
    static_tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    return LaunchDescription([
        visualizer_node,
        rviz_node,
        static_tf_publisher_node
    ])