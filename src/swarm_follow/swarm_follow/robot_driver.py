"""Use current-frame distance control with angular PID as the only history."""

import math

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

from swarm_follow.pid import PID
from swarm_follow.vision import detect_robot

WHEEL_RADIUS = 0.0205   # e-puck, m
AXLE_LENGTH = 0.052     # e-puck, m
MAX_ACCELERATION = 0.10  # m/s² at each wheel rim; enforced by Webots motors.
# Desired PCB elevation above the horizon, in radians (not metres).
TARGET_ANGULAR_HEIGHT = 0.14
SIZE_GAIN = 0.06
# Start yielding well before the following distance to allow motor braking.
# These are visual elevations in radians, not measured distances.
AVOID_START_HEIGHT = 0.045
AVOID_STOP_HEIGHT = 0.09
AVOID_TURN_RATE = 1.5  # rad/s; turn away, never towards the visual target.
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


def no_following_motion(observation, base_speed, base_curvature):
    """Yield to the current visual target without retaining avoidance state."""
    if observation is None:
        return base_speed, base_speed * base_curvature
    urgency = max(0.0, min(1.0, (
        observation.angular_height - AVOID_START_HEIGHT
    ) / (AVOID_STOP_HEIGHT - AVOID_START_HEIGHT)))
    # A cut-off target has unreliable size: brake and turn away immediately.
    if observation.clipped:
        urgency = 1.0
    # Positive bearing is left. Head-on encounters always pass on the right;
    # this tie-break is a constant, not a remembered avoidance direction.
    direction = -1.0 if observation.bearing >= 0.0 else 1.0
    speed = base_speed * (1.0 - urgency)
    turn = ((1.0 - urgency) * base_speed * base_curvature
            + urgency * direction * AVOID_TURN_RATE)
    return speed, turn


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

        # Report what this robot sees and commands, only for the metrics
        # supervisor. Nothing is ever received back.
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node(f'{name}_state')
        self.state_publisher = self.node.create_publisher(
            Float64MultiArray, f'/{name}/follow_state', 10)

    def _publish_state(self, observation, v, w):
        """Publish [following, detected, bearing, angular_height, v, w]."""
        detected = observation is not None
        following = detected and self.mode != 'no_following'
        message = Float64MultiArray()
        message.data = [
            float(following), float(detected),
            observation.bearing if detected else math.nan,
            observation.angular_height if detected else math.nan,
            float(v), float(w),
        ]
        self.state_publisher.publish(message)

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
        observation = self._current_detection()
        if self.mode == 'no_following':
            v, w = no_following_motion(
                observation, self.base_speed, self.base_curvature)
        elif observation is None:
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
        self._publish_state(observation, v / scale, w / scale)
