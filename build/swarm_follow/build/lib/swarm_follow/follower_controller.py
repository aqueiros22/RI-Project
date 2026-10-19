def main(args=None):
    rclpy.init(args=args)
    node = SwarmFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
