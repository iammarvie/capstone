import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import torchvision
#from torchvision.models.detection import FasterRCNN_ResNet50_FPN_Weights
from torchvision.models.detection import SSDLite320_MobileNet_V3_Large_Weights

import time

class ObjectDetectionNode(Node):

    COCO_INSTANCE_CATEGORY_NAMES = [
        '__background__', 'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
        'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'N/A', 'stop sign',
        'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
        'elephant', 'bear', 'zebra', 'giraffe', 'N/A', 'backpack', 'umbrella', 'N/A',
        'N/A', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
        'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
        'bottle', 'N/A', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana',
        'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut',
        'cake', 'chair', 'couch', 'potted plant', 'bed', 'N/A', 'dining table', 'N/A', 'N/A',
        'toilet', 'N/A', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
        'oven', 'toaster', 'sink', 'refrigerator', 'N/A', 'book', 'clock', 'vase', 'scissors',
        'teddy bear', 'hair drier', 'toothbrush'
    ]

    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 1)
        self.publisher_ = self.create_publisher(Image, 'detection_image', 1)
        self.bridge = CvBridge()
        self.i = 0  # Initialize the image counter

        # Load the Faster R-CNN model with pre-trained weights
        self.model= torchvision.models.detection.ssdlite320_mobilenet_v3_large(weights=SSDLite320_MobileNet_V3_Large_Weights.DEFAULT)
        self.model.eval()
        self.get_logger().info('Object detection model loaded successfully')
        self.subscription

    def listener_callback(self, msg):
        start_time = time.perf_counter()
        try:
            self.get_logger().info('Received an image on image_raw')
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            start_time = time.perf_counter()

            # Preprocess the image and perform object detection
            transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
            image_tensor = transform(cv_image)
            try:
                with torch.no_grad():
                    outputs = self.model([image_tensor])[0]
                #outputs = self.model([image_tensor])[0]
            except Exception as e:
                self.get_logger().error(f'Error during model inference: {e}')
                return

            # Draw bounding boxes and labels
            # Directly draw on the image
            for i, (box, score, label) in enumerate(zip(outputs['boxes'], outputs['scores'], outputs['labels'])):
                if score >= 0.5:
                    x1, y1, x2, y2 = box.int().tolist()
                    label_name = self.COCO_INSTANCE_CATEGORY_NAMES[label.item()]
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            print(f"Elapsed time: {elapsed_time} seconds")

            # Convert OpenCV image to ROS Image message and publish
            try:
                detection_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                self.publisher_.publish(detection_image)
                # Log the number of messages sent
                self.get_logger().info('Publishing processed image on detection_image')
                # print the number of messages sent
                self.get_logger().info(' Processed Image number %d sent' % self.i)
                self.i += 1
            except CvBridgeError as e:
                self.get_logger().error('Failed to convert image: %s' % str(e))
        except Exception as e:
            self.get_logger().error(f'Error in listener_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
