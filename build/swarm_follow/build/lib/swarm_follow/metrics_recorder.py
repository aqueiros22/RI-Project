"""Observe current camera frames without reading or changing control state."""

import csv
from pathlib import Path

import numpy as np

from swarm_follow.vision import detect_robot


class ObservationRecorder:
    """Write one sample per simulation step, with no temporal aggregation."""

    def init(self, webots_node, properties):
        """Open a dedicated file and observe the robot's existing camera."""
        self.robot = webots_node.robot
        self.name = self.robot.getName()
        self.mode = properties['mode']
        self.camera = self.robot.getDevice('camera')
        period = int(self.robot.getBasicTimeStep())
        self.dt = period / 1000.0
        self.camera.enable(period)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.fov = self.camera.getFov()
        path = Path(properties['output_dir']) / f'{self.name}_observations.csv'
        self.file = path.open('x', newline='', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'time_s', 'dt_s', 'robot', 'mode', 'frame_valid', 'visible',
            'bearing_rad', 'angular_height_rad', 'clipped',
        ])

    def step(self):
        """Record this frame only; never send feedback to the controller."""
        image = self.camera.getImage()
        observation = None
        if image is not None:
            pixels = np.frombuffer(image, dtype=np.uint8).reshape(
                self.height, self.width, 4)
            observation = detect_robot(pixels, self.fov)
        self.writer.writerow([
            self.robot.getTime(), self.dt, self.name, self.mode,
            int(image is not None), int(observation is not None),
            observation.bearing if observation else '',
            observation.angular_height if observation else '',
            int(observation.clipped) if observation else '',
        ])

    def stop(self):
        """Close the line-buffered recording on a normal shutdown."""
        self.file.close()
