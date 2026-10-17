"""Launch the swarm with a shared selectable motion mode."""

from datetime import datetime
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


def _description(urdf, plugin_values):
    """Return a URDF string with the given plugin fields filled in."""
    root = ET.parse(urdf).getroot()
    for field, value in plugin_values.items():
        root.find(f'webots/plugin/{field}').text = value
    return ET.tostring(root, encoding='unicode')


def launch_setup(context):
    """Pass the selected mode to every robot's Webots plugin."""
    pkg = get_package_share_directory('swarm_follow')
    mode = LaunchConfiguration('mode').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    robot_description = _description(
        os.path.join(pkg, 'resource', 'robot.urdf'), {'mode': mode})

    run_name = LaunchConfiguration('run_name').perform(context) or (
        f'{mode}_{datetime.now():%Y%m%d-%H%M%S}')
    metrics_description = _description(
        os.path.join(pkg, 'resource', 'metrics.urdf'), {
            'mode': mode,
            'duration': LaunchConfiguration('duration').perform(context),
            'metrics_dir': os.path.abspath(
                LaunchConfiguration('metrics_dir').perform(context)),
            'run_name': run_name,
        })

    # Without a window there is nothing to watch, so run as fast as possible.
    webots = WebotsLauncher(
        world=os.path.join(pkg, 'worlds', 'arena.wbt'),
        mode=(LaunchConfiguration('webots_mode').perform(context)
              if gui else 'fast'),
        gui=gui)

    robot_names = [f'robot{i}' for i in range(1, 6)]
    controllers = [
        WebotsController(
            robot_name=n,
            parameters=[{'robot_description': robot_description}])
        for n in robot_names
    ]
    metrics = WebotsController(
        robot_name='metrics',
        parameters=[{'robot_description': metrics_description}])

    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=webots, on_exit=[EmitEvent(event=Shutdown())]))

    return [webots, *controllers, metrics, shutdown]


def generate_launch_description():
    """Expose the three motion modes, preserving strong turning by default."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='strong_turning',
            choices=['no_following', 'weak_turning', 'strong_turning'],
            description='Base motion, or following with weak/strong turns.'),
        DeclareLaunchArgument(
            'webots_mode', default_value='realtime',
            choices=['realtime', 'fast', 'pause'],
            description='Webots simulation speed mode (with gui:=true).'),
        DeclareLaunchArgument(
            'gui', default_value='true', choices=['true', 'false'],
            description='false: Webots without window or rendering, in fast '
                        'mode.'),
        DeclareLaunchArgument(
            'duration', default_value='0',
            description='Simulated seconds before saving the metrics and '
                        'closing Webots; 0 runs until Ctrl+C.'),
        DeclareLaunchArgument(
            'metrics_dir', default_value='metrics',
            description='Folder for the metrics CSVs, relative to the '
                        'current directory.'),
        DeclareLaunchArgument(
            'run_name', default_value='',
            description='Subfolder of this run; default <mode>_<date-time>.'),
        OpaqueFunction(function=launch_setup),
    ])
