import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self, nodeName, topic):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(
            String,
            topic,
            self.listener_callback,
            10)
        self.subscription
        self.received = None  # prevent unused variable warning

    def listener_callback(self, msg):
        self.received = msg.data
        self.get_logger().info('Received: "%s"' % self.received)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
