import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # Perception action server
    perception_action_server = ExecuteProcess(
        cmd=['ros2', 'run', 'simple_grasping', 'basic_grasping_perception_node', '--ros-args', '-p', 'debug_topics:=true'],
        output='screen'
    )

    # MoveItCpp demo executable
    moveit_PPP_node = Node(
        name="pick_and_place_perception",
        package="moveit2_scripts",
        executable="pick_and_place_perception",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
       
        [ perception_action_server,
        moveit_PPP_node]
    )