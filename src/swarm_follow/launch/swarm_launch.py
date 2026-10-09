import os
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    pkg = get_package_share_directory('swarm_follow')
    urdf = os.path.join(pkg, 'resource', 'robot.urdf')

    webots = WebotsLauncher(world=os.path.join(pkg, 'worlds', 'arena.wbt'))

    robot_names = [f'robot{i}' for i in range(1, 6)]
    controllers = [
        WebotsController(robot_name=n, parameters=[{'robot_description': urdf}])
        for n in robot_names
    ]

    shutdown = RegisterEventHandler(OnProcessExit(
        target_action=webots, on_exit=[EmitEvent(event=Shutdown())]))

    return LaunchDescription([webots, *controllers, shutdown])