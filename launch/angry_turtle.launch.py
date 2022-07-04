import launch
import launch.actions
import launch.substitutions
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    angryturtle = Node(package='angry_turtle', executable='angry_turtle_node', output='screen')
    turtlesim = Node(package='turtlesim', executable='turtlesim_node', output='screen')

    return LaunchDescription([
        turtlesim,
        angryturtle,
    ])