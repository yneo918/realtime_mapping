from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    interactive_flag = LaunchConfiguration('interactive').perform(context)

    node_arguments = ['--config', config_file]
    if interactive_flag.lower() in ('true', '1', 'yes'):
        node_arguments.append('--interactive')

    realtime_mapper_node = Node(
        package='realtime_mapping',
        executable='realtime_mapper',
        name='realtime_mapper',
        arguments=node_arguments,
        output='screen'
    )

    return [realtime_mapper_node]


def generate_launch_description():
    # Locate the package share directory
    pkg_dir = get_package_share_directory('realtime_mapping')

    # Define launch file arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'message_config.yaml'),
        description='Path to the mapping configuration file'
    )

    interactive_arg = DeclareLaunchArgument(
        'interactive',
        default_value='false',
        description='Enable interactive topic selection'
    )

    return LaunchDescription([
        config_file_arg,
        interactive_arg,
        OpaqueFunction(function=_launch_setup)
    ])
