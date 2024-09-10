# Camera Publisher node

import cv2

# The ROS2 packagaes and  modules
import rclpy
from sensor_msgs.msgs import Image
from rclpy.node import Node
from cv_bridge import CvBridge

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
		self.topicNameFrames = 'topic_camera_image'

		#the queue size for messages
		self.queueSize=20
		
		#self create publisher  creates the publisher that 
