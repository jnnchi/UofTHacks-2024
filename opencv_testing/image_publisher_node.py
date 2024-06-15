import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import imutils

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.br = CvBridge()

    def publish_image(self, cv_image):
        ros_image = self.br.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.publisher_.publish(ros_image)
        self.get_logger().info('Publishing image')

def capture_image(url):
    img_resp = requests.get(url)
    img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8)
    img = cv2.imdecode(img_arr, -1)
    return img

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    url = "http://YOUR_IP_WEBCAM_APP_IP_HERE:8080/shot.jpg"
    print("Press 'c' to capture an image from the IP Webcam, or 'q' to quit.")

    while True:
        key = input("Press 'c' to capture or 'q' to quit: ")

        if key == 'c':
            img = capture_image(url)
            img = imutils.resize(img, width=800, height=600)
            cv2.imshow("Captured Image", img)
            image_publisher.publish_image(img)
            
            print("Press any key in the image window to continue.")
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        elif key == 'q':
            print("Exiting program.")
            break
        else:
            print("Invalid input. Please press 'c' to capture or 'q' to quit.")

    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
