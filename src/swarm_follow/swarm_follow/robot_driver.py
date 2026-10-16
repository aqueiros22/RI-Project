"""Use current-frame distance control with angular PID as the only history."""

import math

import numpy as np

from swarm_follow.pid import PID
from swarm_follow.vision import detect_robot

WHEEL_RADIUS = 0.0205   # e-puck, m
AXLE_LENGTH = 0.052     # e-puck, m
MAX_ACCELERATION = 0.10  # m/s² at each wheel rim; enforced by Webots motors.
# Desired PCB elevation above the horizon, in radians (not metres).
TARGET_ANGULAR_HEIGHT = 0.14
SIZE_GAIN = 0.06
# (Kp, Ki, Kd): initial settings to tune in Webots.
TURNING_PID_GAINS = {
    'no_following': (0.0, 0.0, 0.0),
    'weak_turning': (0.5, 0.02, 0.05),
    'strong_turning': (2.0, 0.05, 0.15),
}

# Each robot starts with a different linear speed and curvature.
BASE_MOTION = {
    'robot1': (0.050, 1.50),
    'robot2': (0.060, 1.75),
    'robot3': (0.070, 2.00),
    'robot4': (0.080, 2.25),
    'robot5': (0.090, 2.50),
}


def following_speed(observation):
    """Return uncapped forward speed using only this observation."""
    # Smaller apparent height means a more distant target. This is a visual
    # distance proxy, not a measurement in metres. No previous frame is used.
    speed = 0.0
    if not observation.clipped:
        speed = max(0.0, SIZE_GAIN * (
            TARGET_ANGULAR_HEIGHT / observation.angular_height - 1))
        speed *= max(0.0, math.cos(observation.bearing))
    return speed


class RobotDriver:
    """Keep history only in the PID; let motors handle acceleration."""

    def init(self, webots_node, properties):
        """Enable the RGB camera and initialize wheel velocity control."""
        self.mode = properties.get('mode', 'strong_turning')
        if self.mode not in TURNING_PID_GAINS:
            raise ValueError(f'Unknown motion mode: {self.mode!r}')
        self.turning_pid = PID(*TURNING_PID_GAINS[self.mode])
        self.robot = webots_node.robot
        name = self.robot.getName()

        self.left = self.robot.getDevice('left wheel motor')
        self.right = self.robot.getDevice('right wheel motor')
        self.left.setPosition(float('inf'))
        self.right.setPosition(float('inf'))
        self.left.setAcceleration(MAX_ACCELERATION / WHEEL_RADIUS)
        self.right.setAcceleration(MAX_ACCELERATION / WHEEL_RADIUS)
        self.left_max_velocity = self.left.getMaxVelocity()
        self.right_max_velocity = self.right.getMaxVelocity()
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

        self.base_speed, self.base_curvature = BASE_MOTION.get(
            name, (0.05, 0.2))
        self.camera = self.robot.getDevice('camera')
        period = int(self.robot.getBasicTimeStep())
        self.dt = period / 1000.0
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

    def _wheel_scale(self, v, w):
        """Calculate actuator saturation from this step's requested motion."""
        vl = (v - w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        vr = (v + w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        # Leave one representable float below the Webots limit. Without this
        # margin, division/rounding can turn a value equal to maxVelocity into
        # a slightly larger command and produce a simulator warning.
        left_limit = math.nextafter(self.left_max_velocity, 0.0)
        right_limit = math.nextafter(self.right_max_velocity, 0.0)
        return max(1.0, abs(vl) / left_limit, abs(vr) / right_limit)

    def step(self):
        """Request current-frame forward motion and a PID-controlled turn."""
        observation = (self._current_detection()
                       if self.mode != 'no_following' else None)
        if observation is None:
            self.turning_pid.reset()
            v = self.base_speed
            w = self.base_speed * self.base_curvature
        else:
            v = following_speed(observation)
            w = self.turning_pid.update(
                observation.bearing, self.dt, float('inf'),
                is_saturated=lambda turn: self._wheel_scale(v, turn) > 1.0)
        vl = (v - w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        vr = (v + w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        # Respect actual actuator limits, not an extra controller speed cap.
        # Scale both wheels together to preserve requested curvature.
        scale = self._wheel_scale(v, w)
        self.left.setVelocity(vl / scale)
        self.right.setVelocity(vr / scale)
