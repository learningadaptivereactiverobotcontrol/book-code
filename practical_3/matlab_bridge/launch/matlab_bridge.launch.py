import os.path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulated = DeclareLaunchArgument("simulated", default_value="False")

    bridge_node = Node(
        package="matlab_bridge",
        executable="matlab_bridge",
        parameters=[{
            "simulated": LaunchConfiguration("simulated")
        }],
        output="both"
    )

    obstacle_bridge_node = Node(
        package="matlab_bridge",
        executable="ObstacleZMQBridge.py",
        name='obstacle_zmq_bridge',
        output="both",
        parameters=[]
    )

    path_bridge_node = Node(
        package="matlab_bridge",
        executable="PathZMQBridge.py",
        name='path_zmq_bridge',
        output="both",
        parameters=[]
    )


    nodes = [
        bridge_node,
        obstacle_bridge_node,
        path_bridge_node
    ]

    return LaunchDescription([simulated] + nodes)
