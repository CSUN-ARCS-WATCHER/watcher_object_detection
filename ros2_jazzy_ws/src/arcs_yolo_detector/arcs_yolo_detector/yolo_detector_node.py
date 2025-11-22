#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2

# Check if YOLO is available
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('input_topic', '/camera/rgb/image')
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        input_topic = self.get_parameter('input_topic').value
        
        # Check if YOLO is available
        if not YOLO_AVAILABLE:
            self.get_logger().error('Ultralytics not installed! Run: pip3 install ultralytics')
            return
        
        # Initialize YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        
        # Create subscriber to image topic
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # Create publisher for detections
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo_detections',
            10
        )
        
        # Optional: Publisher for annotated images
        self.annotated_pub = self.create_publisher(
            Image,
            '/yolo_annotated',
            10
        )
        
        self.get_logger().info(f'YOLO Detector initialized!')
        self.get_logger().info(f'Listening on: {input_topic}')
        self.get_logger().info(f'Publishing to: /yolo_detections')
        
        # Counter for debugging
        self.frame_count = 0
    
    def image_callback(self, msg):
        """Called every time a new image arrives"""
        
        self.frame_count += 1
        
        # Log every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Processed {self.frame_count} frames')
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO inference
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # Process each detection
            for result in results:
                for box in result.boxes:
                    detection = Detection2D()
                    
                    # Extract bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    # Class and confidence
                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = str(int(box.cls))
                    hyp.hypothesis.score = float(box.conf)
                    detection.results.append(hyp)
                    
                    detection_array.detections.append(detection)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Optional: Publish annotated image
            annotated = results[0].plot()
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
