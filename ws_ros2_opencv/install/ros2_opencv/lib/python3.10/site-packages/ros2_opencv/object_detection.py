import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import torchvision
from torchvision.models.detection import FasterRCNN_ResNet50_FPN_Weights

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
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 20)
        self.publisher_ = self.create_publisher(Image, 'detection_image', 20)
        self.bridge = CvBridge()
        self.i = 0  # Initialize the image counter

        # Load the Faster R-CNN model with pre-trained weights
        #self.model = torchvision.models.detection.fasterrcnn_resnet50_fpn(weights=FasterRCNN_ResNet50_FPN_Weights.DEFAULT)
        #self.model.eval()
        self.model = torchvision.models.detection.ssdlite320_mobilenet_v3_large(weights=ssdlite320_mobilenet_v3_large_Weights.DEFAULT)
        self.model.eval()
        self.get_logger().info('Object detection model loaded successfully')
        self.subscription

    def listener_callback(self, msg):
        try:
            self.get_logger().info('Received an image on image_raw')
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Preprocess the image and perform object detection
            transform = torchvision.transforms.Compose([torchvision.transforms.ToTensor()])
            image_tensor = transform(cv_image)
            try:
                # Process every 5th frame
                if self.i % 5 == 0:  
                    outputs = self.model([image_tensor])[0]
                else:
                    # Skip this frame, return early
                    self.get_logger().info('Skipping frame %d' % self.i)
                    self.i += 1
                    return

                self.i += 1

            except Exception as e:
                self.get_logger().error(f'Error during model inference: {e}')
                return
                
            # Draw bounding boxes and labels
            for i, (box, score, label) in enumerate(zip(outputs['boxes'], outputs['scores'], outputs['labels'])):
                if score >= 0.5:
                    x1, y1, x2, y2 = box.int().tolist()
                    label_name = self.COCO_INSTANCE_CATEGORY_NAMES[label.item()]  # Get the label from the list
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    overlay = cv_image.copy()
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), -1)
                    alpha = 0.4
                    cv_image = cv2.addWeighted(overlay, alpha, cv_image, 1 - alpha, 0)
                    cv2.putText(cv_image, label_name, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
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
