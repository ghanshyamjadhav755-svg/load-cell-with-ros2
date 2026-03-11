from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('load_cell_pkg'),
        'config',
        'load_cell_params.yaml'
    )
    load_cell_data_node = Node(
        package='load_cell_pkg',
        executable='load_cell_data',
        name='load_cell_data',
        output='screen'
    )

    load_cell_delivery_node = Node(
        package='load_cell_pkg',
        executable='load_cell_delivery',
        name='load_cell_delivery',
        parameters=[config],
        output='screen'
    )

    return LaunchDescription([
        load_cell_data_node,
        load_cell_delivery_node
    ])