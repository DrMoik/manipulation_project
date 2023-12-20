import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Perception action server
    perception_action_server = ExecuteProcess(
        cmd=['ros2', 'run', 'simple_grasping', 'basic_grasping_perception_node', '--ros-args', '-p', 'debug_topics:=true'],
        output='screen'
    )

    # get_pose_client node
    get_pose_client_node = Node(
        package='get_cube_pose',
        executable='get_pose_client',
        name='get_pose_client',
        output='screen'
    )

    return LaunchDescription([
        perception_action_server,
        get_pose_client_node
    ])
