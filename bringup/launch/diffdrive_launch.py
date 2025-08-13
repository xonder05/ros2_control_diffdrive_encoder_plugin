from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    controller_config_file = os.path.join(
        get_package_share_directory('ros2_control_diffdrive_encoder_plugin'),
        'config',
        'controllers.yaml'
    )

    manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config_file],
        remappings=[
            ('/diff_drive_controller/cmd_vel', '/cmd_vel'),
            ('/controller_manager/robot_description', '/robot_description')
        ]
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager-timeout", "60"],
    )

    return LaunchDescription([
        manager_node,
        controller_spawner,
    ])
