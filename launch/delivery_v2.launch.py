from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('load_cell_pkg'),
        'config',
        'dock_manager.yaml'
    )
    config2 = os.path.join(
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
    nav2_dockv2_node = Node(
        package='load_cell_pkg',
        executable='nav2_dockv2',
        name='nav2_dockv2',
        parameters=[config2],
        output='screen'
    )
    dock_manager_node = Node(
        package='load_cell_pkg',
        executable='dock_manager',
        name='dock_manager',
        parameters=[config],
        output='screen'
    )


    return LaunchDescription([
        load_cell_data_node,
        dock_manager_node,
        nav2_dockv2_node
    ])