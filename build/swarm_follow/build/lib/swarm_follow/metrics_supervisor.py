"""
Observe the swarm and write the run metrics to CSV.

This Webots supervisor only reads: true poses from the simulator and the
state each robot publishes. It never sends anything to the robots, so their
control stays purely reactive.
"""

import csv
import math
import os

import rclpy
from std_msgs.msg import Float64MultiArray

from swarm_follow.metrics import MetricsTracker, SUMMARY_FIELDS

ROBOT_NAMES = [f'robot{i}' for i in range(1, 6)]
TIMESERIES_FIELDS = [
    't', 'robot', 'x', 'y', 'yaw', 'following', 'detected', 'bearing',
    'angular_height', 'v', 'w',
]
SUMMARY_PERIOD = 1.0  # s of simulated time between summary rewrites.


class MetricsSupervisor:
    """Record poses and robot reports; save timeseries and summary CSVs."""

    def init(self, webots_node, properties):
        """Find the robots, subscribe to their states and open the CSVs."""
        self.robot = webots_node.robot
        self.mode = properties.get('mode', '')
        self.seed = int(properties.get('seed', '0'))
        self.duration = float(properties.get('duration', '0'))
        run_dir = os.path.join(
            properties.get('metrics_dir', 'metrics'),
            properties.get('run_name', 'run'))
        os.makedirs(run_dir, exist_ok=True)
        self.summary_path = os.path.join(run_dir, 'summary.csv')

        self.nodes = {}
        for name in ROBOT_NAMES:
            node = self.robot.getFromDef(name.upper())
            if node is None:
                raise RuntimeError(f'DEF {name.upper()} not found in world')
            self.nodes[name] = node

        dt = self.robot.getBasicTimeStep() / 1000.0
        self.tracker = MetricsTracker(ROBOT_NAMES, dt)
        self.states = {name: None for name in ROBOT_NAMES}

        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros = rclpy.create_node('metrics_supervisor')
        for name in ROBOT_NAMES:
            self.ros.create_subscription(
                Float64MultiArray, f'/{name}/follow_state',
                lambda message, name=name: self._on_state(name, message), 10)

        self.timeseries_file = open(
            os.path.join(run_dir, 'timeseries.csv'), 'w', newline='')
        self.timeseries = csv.writer(self.timeseries_file)
        self.timeseries.writerow(TIMESERIES_FIELDS)
        self.next_summary = SUMMARY_PERIOD
        self.finished = False
        self.ros.get_logger().info(f'Writing metrics to {run_dir}')

    def _on_state(self, name, message):
        self.states[name] = list(message.data)

    def _pose(self, name):
        x, y, _ = self.nodes[name].getPosition()
        rotation = self.nodes[name].getOrientation()
        return x, y, math.atan2(rotation[3], rotation[0])

    def step(self):
        """Record one simulation step."""
        if self.finished:
            return
        # Handle every message that arrived since the last step.
        for _ in range(4 * len(ROBOT_NAMES)):
            rclpy.spin_once(self.ros, timeout_sec=0)

        t = self.robot.getTime()
        poses = {name: self._pose(name) for name in ROBOT_NAMES}
        reports = {}
        for name in ROBOT_NAMES:
            state = self.states[name]
            empty = [math.nan] * 6
            following, detected, bearing, height, v, w = state or empty
            if state is not None:
                reports[name] = (following == 1.0, bearing)
            x, y, yaw = poses[name]
            self.timeseries.writerow([
                f'{t:.3f}', name, f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}',
                _flag(following), _flag(detected), _number(bearing),
                _number(height), _number(v), _number(w),
            ])
        self.tracker.update(poses, reports)

        if t >= self.next_summary:
            self._write_summary()
            self.next_summary += SUMMARY_PERIOD
        if self.duration > 0 and t >= self.duration:
            self._write_summary()
            self.timeseries_file.close()
            self.finished = True
            self.ros.get_logger().info('Metrics saved, closing Webots.')
            self.robot.simulationQuit(0)

    def _write_summary(self):
        self.timeseries_file.flush()
        with open(self.summary_path, 'w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=SUMMARY_FIELDS)
            writer.writeheader()
            writer.writerows(self.tracker.summary_rows(self.mode, self.seed))


def _flag(value):
    return '' if math.isnan(value) else int(value)


def _number(value):
    return '' if math.isnan(value) else f'{value:.4f}'
