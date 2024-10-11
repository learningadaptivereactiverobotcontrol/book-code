import os.path
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulated = DeclareLaunchArgument("simulated", default_value="False")

    obstacle_bridge_node = Node(
        package="matlab_bridge",
        executable="ObstacleZMQBridge.py",
        name='obstacle_zmq_bridge',
        output="both",
        parameters=[]
    )

    mppi_controller_node = Node(
        package="matlab_bridge",
        executable="MppiController.py",
        name='integrator_switching',
        output="both",
        parameters=[]
    )

    obs_streamer_node = Node(
        package="matlab_bridge",
        executable="ObstacleStreamerBridge.py",
        name='obs_streamer',
        output="both",
        parameters=[]
    )

    nodes = [
        obstacle_bridge_node,
        mppi_controller_node,
        obs_streamer_node
    ]

    return LaunchDescription([simulated] + nodes)
