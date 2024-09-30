import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import torch
import time

class ObjectDetectionNode(Node):

    def __init__(self):
        super().__init__('object_detection_node')
        self.bridge = CvBridge()  # Initialize CvBridge
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 1)
        self.publisher_ = self.create_publisher(Image, 'detection_image', 1)
        self.i = 0  # Initialize the image counter

        # Load the YOLO model and set it to GPU if available
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f'Using device: {device}')

        # Load custom YOLOv5 model
        self.model = YOLO('yolov5nu.pt')
        self.model = self.model.to(device)  # Move model to GPU if available

    def listener_callback(self, msg):
        start_time = time.perf_counter()
        try:
            self.get_logger().info('Received an image on image_raw')

            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Convert the image to a PyTorch tensor and move it to GPU
            img_tensor = torch.from_numpy(cv_image).float().permute(2, 0, 1).unsqueeze(0).to('cuda' if torch.cuda.is_available() else 'cpu')

            # Perform object detection with YOLOv5
            with torch.no_grad():
                results = self.model(img_tensor)  # Run YOLO on the image

            predictions = results[0]  # Get the first batch of predictions

            for box in predictions.boxes:
                # Extract coordinates, confidence, and label
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = box.conf[0].cpu().item()  # Confidence score
                class_id = int(box.cls[0].cpu())  # Class index
                label = self.model.names[class_id]  # Convert class index to label

                # Only draw boxes if confidence is above threshold
                if conf > 0.8: ### CHANGE BACK TO 0.6
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(cv_image, f'{label} {conf:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)

        except Exception as e:
            self.get_logger().error(f'Error during model inference: {e}')
            return

        end_time = time.perf_counter()
        elapsed_time = end_time - start_time
        print(f"Elapsed time: {elapsed_time} seconds")

        # Convert OpenCV image to ROS Image message and publish
        try:
            detection_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            self.publisher_.publish(detection_image)
            self.get_logger().info('Publishing processed image on detection_image')
            self.get_logger().info(f'Processed Image number {self.i} sent')
            self.i += 1
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
