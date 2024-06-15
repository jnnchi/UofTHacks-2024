import rclpy, random
from pub import MinimalPublisher

rclpy.init(args=None)

publisher = MinimalPublisher(nodeName="shareBoardSub", topic="shareBoard") #create node

rate = publisher.create_rate(1)
#move = "c2d3"
while rclpy.ok():
    move = input("Enter move")
    publisher.timer_callback(move)
    #rate.sleep()

rclpy.spin_once(publisher)
publisher.destroy_node()
rclpy.shutdown()