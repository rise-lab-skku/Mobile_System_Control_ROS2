import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    object_name_arg = DeclareLaunchArgument(
        'object_name',
        default_value='ego',
        description='Short name of the object definition file in the config folder (e.g., ego)'
    )

    objects_definition_file_path = [
        FindPackageShare('carla_spawn_objects'),
        TextSubstitution(text='/config/'),
        LaunchConfiguration('object_name'),
        TextSubstitution(text='.json')
    ]

    carla_spawn_objects_node = Node(
        package='carla_spawn_objects',
        executable='carla_spawn_objects',
        name='carla_spawn_objects',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'objects_definition_file': objects_definition_file_path
        }]
    )

    return LaunchDescription([
        object_name_arg,
        carla_spawn_objects_node
    ])