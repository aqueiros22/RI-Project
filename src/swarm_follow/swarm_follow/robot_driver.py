"""Control each robot using only its current frontal camera observations."""

import math

import numpy as np

from swarm_follow.vision import detect_robot

WHEEL_RADIUS = 0.0205   # e-puck, m
AXLE_LENGTH = 0.052     # e-puck, m
MAX_WHEEL = 6.28        # rad/s
# Desired PCB elevation above the horizon, in radians (not metres).
TARGET_ANGULAR_HEIGHT = 0.14
SIZE_GAIN = 0.06
TURNING_GAINS = {
    'no_following': 0.0,
    'weak_turning': 0.5,
    'strong_turning': 2.0,
}
MAX_SPEED = 0.10

# Each robot starts with a different linear speed and curvature.
BASE_MOTION = {
    'robot1': (0.050, 1.50),
    'robot2': (0.060, 1.75),
    'robot3': (0.070, 2.00),
    'robot4': (0.080, 2.25),
    'robot5': (0.090, 2.50),
}


class RobotDriver:
    """Run one independent, memoryless follower per robot."""

    def init(self, webots_node, properties):
        """Enable the RGB camera and initialize wheel velocity control."""
        self.mode = properties.get('mode', 'strong_turning')
        if self.mode not in TURNING_GAINS:
            raise ValueError(f'Unknown motion mode: {self.mode!r}')
        self.bearing_gain = TURNING_GAINS[self.mode]
        self.robot = webots_node.robot
        name = self.robot.getName()

        self.left = self.robot.getDevice('left wheel motor')
        self.right = self.robot.getDevice('right wheel motor')
        self.left.setPosition(float('inf'))
        self.right.setPosition(float('inf'))
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

        self.base_speed, self.base_curvature = BASE_MOTION.get(
            name, (0.05, 0.2))
        self.v = self.base_speed
        self.w = self.base_speed * self.base_curvature
        self.camera = self.robot.getDevice('camera')
        period = int(self.robot.getBasicTimeStep())
        self.camera.enable(period)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.field_of_view = self.camera.getFov()

    def _current_detection(self):
        """Detect a robot from this frame's BGRA pixels only."""
        image = self.camera.getImage()
        if image is None:
            return None
        pixels = np.frombuffer(image, dtype=np.uint8).reshape(
            self.height, self.width, 4)
        return detect_robot(pixels, self.field_of_view)

    def step(self):
        """Follow a visible robot or immediately resume base motion."""
        observation = (self._current_detection()
                       if self.mode != 'no_following' else None)
        if observation is None:
            self.v = self.base_speed
            self.w = self.base_speed * self.base_curvature
        else:
            bearing = observation.bearing
            self.v = min(MAX_SPEED, max(
                0.0, SIZE_GAIN * (
                    TARGET_ANGULAR_HEIGHT / observation.angular_height - 1)))
            self.v *= max(0.0, math.cos(bearing))
            # A cropped target has an unreliable size: turn to recover it.
            if observation.clipped:
                self.v = 0.0
            self.w = self.bearing_gain * bearing
        vl = (self.v - self.w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        vr = (self.v + self.w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        # Scale both wheels together to preserve commanded curvature.
        scale = max(1.0, abs(vl) / MAX_WHEEL, abs(vr) / MAX_WHEEL)
        self.left.setVelocity(vl / scale)
        self.right.setVelocity(vr / scale)
