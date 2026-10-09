WHEEL_RADIUS = 0.0205   # e-puck, m
AXLE_LENGTH = 0.052     # e-puck, m
MAX_WHEEL = 6.28        # rad/s

# Each robot starts with a different linear speed and curvature.
BASE_MOTION = {
    'robot1': (0.050, 0.20),
    'robot2': (0.060, 0.30),
    'robot3': (0.070, 0.40),
    'robot4': (0.080, 0.50),
    'robot5': (0.090, 0.60),
}


class RobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        name = self.robot.getName()

        self.left = self.robot.getDevice('left wheel motor')
        self.right = self.robot.getDevice('right wheel motor')
        self.left.setPosition(float('inf'))
        self.right.setPosition(float('inf'))
        self.left.setVelocity(0.0)
        self.right.setVelocity(0.0)

        self.v, self.w = BASE_MOTION.get(name, (0.05, 0.2))

    def step(self):
        vl = (self.v - self.w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        vr = (self.v + self.w * AXLE_LENGTH / 2) / WHEEL_RADIUS
        self.left.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, vl)))
        self.right.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, vr)))