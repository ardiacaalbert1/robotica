import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class LandscapeDrawer(Node):
    def __init__(self):
        super().__init__('landscape_drawer')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

    def draw_square(self):
        twist = Twist()
        twist.linear.x = 2.0
        twist.angular.z = 1.0
        for _ in range(4):
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=1)  # Wait for a second
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=1)  # Wait for a second

def main(args=None):
    rclpy.init(args=args)
    drawer = LandscapeDrawer()
    try:
        drawer.draw_square()
    finally:
        drawer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
