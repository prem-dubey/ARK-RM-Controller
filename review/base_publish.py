import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Ekfnode(Node):
    def __init__(self):
        super().__init__('publisher')

        # Publisher
        self.publisher_ = self.create_publisher(
            String,
            'topic',
            10
        )

        # Timer
        self.timer_ = self.create_timer(
            0.5,
            self.timer_callback
        )

        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello String {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = Ekfnode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

