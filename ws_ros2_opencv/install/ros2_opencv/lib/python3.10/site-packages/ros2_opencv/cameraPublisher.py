# Camera Publisher node

import cv2

# The ROS2 packagaes and  modules
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import time

#The argument "Node" means that the PublisherNodeClass is a child of the class called Node
# The Node class is a standard ROS2 class

class PublisherNodeClass(Node):

# constructor
	def __init__(self):

		#initialize the attributes of the parent class
		super().__init__('publisher_node')
		#create an instance of the videocapture
		# use the camera number "0" . there are more saved on my phone incase 0 does not work
		self.cameraDeviceNumber = 0
		self.camera = cv2.VideoCapture(self.cameraDeviceNumber)

		#cvbridge converts images to ros2 messages
		self.bridgeObject = CvBridge()

		# name of topic used to transfer camera images
		self.topicNameFrames = 'image_raw'#'topic_camera_image'

		#the queue size for messages
		self.queueSize=1
		
		#self create publisher  creates the publisher that publishes the messages of the type Image, over the topic self.topicNameFrames and with the queue size self.queueSize
		self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)

		#communication period
		self.periodCommunication = 0.1

		#create the timer that calls the function self.timer_callback at the period self.periodCommunication
		self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)

		# this is the counter that is used to count the number of messages sent
		self.i = 0
		# this is the callback function that is called every self.periodCommunication seconds
	def timer_callbackFunction(self):

		# read the image from the camera
		success, frame = self.camera.read()
		# resize the image
		frame = cv2.resize(frame, (320, 320), interpolation=cv2.INTER_AREA)
		#flip the image
		frame = cv2.flip(frame,-1)
		# IF the image is read successfully
		if success:
			# convert the image to a ros2 message
			imageMessage = self.bridgeObject.cv2_to_imgmsg(frame,'bgr8')

			# publish the message
			self.publisher.publish(imageMessage)

		# print the number of messages sent
		self.get_logger().info('Image number %d sent' % self.i)
		self.i += 1

# the main function

def main(args=None):
	#initialize the ROS2 system
	rclpy.init(args=args)

	#create an instance of the class PublisherNodeClass
	publisher_node = PublisherNodeClass()

	#keep the node running
	rclpy.spin(publisher_node)

	#destroy the node
	publisher_node.destroy_node()
	
	#shutdown the ROS2 system
	rclpy.shutdown()

# call the main function

if __name__ == '__main__':
	main()
