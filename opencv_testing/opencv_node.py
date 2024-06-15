import rclpy
from rclpy.node import Node

class OpenCVNode(Node):
    def __init__(self):
        super().__init__("OpenCV Node")
        self.board_imgs_topic_sub_ = self.create_subscription(ImageType, '/boardimagetopicNAME', self.img_callback, 10)

    def img_callback(self, msg: ImageType):
        # should do all the img processing here
        self.get_logger().info(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVNode()
    rclpy.spin(node)
    rclpy.shutdown()


# stuff from chatgpt
class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Now you can use OpenCV to process cv_image
        # ...

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
