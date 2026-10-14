"""Control each robot using only its current frontal camera observations."""

import math

WHEEL_RADIUS = 0.0205   # e-puck, m
AXLE_LENGTH = 0.052     # e-puck, m
MAX_WHEEL = 6.28        # rad/s
SENSOR_RANGE = 4.0
HALF_FOV = math.radians(50)
FOLLOW_DISTANCE = 0.15  # camera-to-marker horizontal distance, m
DISTANCE_GAIN = 0.5
BEARING_GAIN = 2.0
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
        """Enable local recognition and initialize wheel velocity control."""
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
        self.camera = self.robot.getDevice('cam')
        period = int(self.robot.getBasicTimeStep())
        self.camera.enable(period)
        self.camera.recognitionEnable(period)

    def _current_detection(self):
        """Return the nearest valid marker's distance and bearing this frame."""
        nearest = None
        for obj in self.camera.getRecognitionObjects():
            if obj.getModel() != 'swarm_robot':
                continue
            # Webots camera coordinates: X forward, Y left, Z up.
            forward, left, up = obj.getPosition()
            if not all(math.isfinite(value) for value in (forward, left, up)):
                continue
            distance = math.hypot(forward, left)
            bearing = math.atan2(left, forward)
            if (forward <= 0 or distance > SENSOR_RANGE or
                    abs(bearing) > HALF_FOV):
                continue
            if nearest is None or distance < nearest[0]:
                nearest = (distance, bearing)
        return nearest

    def step(self):
        """Follow a visible robot or immediately resume base motion."""
        observation = self._current_detection()
        if observation is None:
            self.v = self.base_speed
            self.w = self.base_speed * self.base_curvature
        else:
            distance, bearing = observation
            self.v = min(MAX_SPEED, max(
                0.0, DISTANCE_GAIN * (distance - FOLLOW_DISTANCE)))
            self.v *= max(0.0, math.cos(bearing))
            self.w = BEARING_GAIN * bearing
        vl = (self.v - self.w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        vr = (self.v + self.w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        # Scale both wheels together to preserve commanded curvature.
        scale = max(1.0, abs(vl) / MAX_WHEEL, abs(vr) / MAX_WHEEL)
        self.left.setVelocity(vl / scale)
        self.right.setVelocity(vr / scale)
