"""Launch the swarm with a shared selectable motion mode."""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration

from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def launch_setup(context):
    """Pass the selected mode to every robot's Webots plugin."""
    pkg = get_package_share_directory('swarm_follow')
    urdf = os.path.join(pkg, 'resource', 'robot.urdf')
    robot_description = ET.parse(urdf).getroot()
    robot_description.find('webots/plugin/mode').text = (
        LaunchConfiguration('mode').perform(context))
    robot_description = ET.tostring(robot_description, encoding='unicode')

    webots = WebotsLauncher(world=os.path.join(pkg, 'worlds', 'arena.wbt'))

    robot_names = [f'robot{i}' for i in range(1, 6)]
    controllers = [
        WebotsController(
            robot_name=n,
            parameters=[{'robot_description': robot_description}])
        for n in robot_names
    ]

    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=webots, on_exit=[EmitEvent(event=Shutdown())]))

    return [webots, *controllers, shutdown]


def generate_launch_description():
    """Expose the three motion modes, preserving strong turning by default."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='strong_turning',
            choices=['no_following', 'weak_turning', 'strong_turning'],
            description='Base motion, or following with weak/strong turns.'),
        OpaqueFunction(function=launch_setup),
    ])
