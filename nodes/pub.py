import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self, nodeName, topic):
        super().__init__(nodeName) #node name
        self.publisher_ = self.create_publisher(String, topic, 10) #declares the node publishes messages of type String over a topic named 'topic'
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # creates message with counter value appended
    def timer_callback(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data) #publishes to console
        self.i += 1


def main(args=None):
    rclpy.init(args=args) #init library

    minimal_publisher = MinimalPublisher() #create node

    rclpy.spin(minimal_publisher) #spins node

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()