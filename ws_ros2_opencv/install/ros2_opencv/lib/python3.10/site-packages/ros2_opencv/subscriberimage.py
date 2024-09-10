# Camera Subscriber node

# The ROS2 packages and modules
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2

# The argument "Node" means that the SubscriberNodeClass is a child of the class called Node
# The Node class is a standard ROS2 class

class SubscriberNodeClass(Node):

    # constructor
    def __init__(self):

        # initialize the attributes of the parent class
        super().__init__('subscriber_node')

        # cvbridge converts images to ros2 messages
        self.bridgeObject = CvBridge()

        # name of topic used to transfer camera images
        self.topicNameFrames = 'topic_camera_image'

        # the queue size for messages
        self.queueSize = 20

        # self create subscriber creates the subscriber that subscribes to the messages of the type Image, over the topic self.topicNameFrames and with the queue size self.queueSize
        self.subscription = self.create_subscription(Image, self.topicNameFrames, self.listener_callback, self.queueSize)
        self.subscription # prevent unused variable warning

        # this is the callback function that is called every time a message is received
    def listener_callback(self, msg):

        self.get_logger().info('The image has been received')
        # convert the ros2 message to an image
        frame = self.bridgeObject.imgmsg_to_cv2(msg)

        # show the image
        cv2.imshow('Camera Video', frame)
        cv2.waitKey(1)

# main function
def main(args=None):
    rclpy.init(args=args)

    # create an instance of the class SubscriberNodeClass
    subscriberNode = SubscriberNodeClass()

    # keep the node running
    rclpy.spin(subscriberNode)

    # destroy the node explicitly
    subscriber_node.destroy_node()
    rclpy.shutdown()

# if the python file is run directly
if __name__ == '__main__':
    main()
