import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        
        # Load the optimized ONNX model for high-speed tracking
        self.model = YOLO('/home/anujjj_k/drone_ws/src/human_detection/human_detection/best.onnx', task='detect')
        self.bridge = CvBridge()
        
        # Subscribe to Gazebo's raw camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers: One for math (geo-tagging), one for visuals (debugging)
        self.math_pub = self.create_publisher(Detection2DArray, '/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/camera/detections_debug', 10)
        
        self.get_logger().info("Vision tracking node initialized! Waiting for frames...")

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 2. Run the Predictive Tracking
        # Enforcing the proposal constraints: 60% confidence, 0.5 NMS, class 0 (person only)
        results = self.model.track(
            cv_image, 
            persist=True, 
            conf=0.35, 
            iou=0.5, 
            classes=[0], 
            verbose=False
        )
        
        # 3. Publish the math for the Geo-Tagging node
        det_array = Detection2DArray()
        det_array.header = msg.header
        
        if results[0].boxes.id is not None:
            boxes = results[0].boxes.xywh.cpu().numpy()
            track_ids = results[0].boxes.id.int().cpu().tolist()
            confs = results[0].boxes.conf.cpu().tolist()
            
            for box, track_id, conf in zip(boxes, track_ids, confs):
                x_center, y_center, width, height = box
                
                det = Detection2D()
                det.header = msg.header
                det.bbox.center.position.x = float(x_center)
                det.bbox.center.position.y = float(y_center)
                det.bbox.size_x = float(width)
                det.bbox.size_y = float(height)
                
                # Attach the tracking ID as a string so the geo-tagging script knows who is who
                det.id = str(track_id)
                
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(0)
                hypothesis.hypothesis.score = float(conf)
                det.results.append(hypothesis)
                
                det_array.detections.append(det)
                
        self.math_pub.publish(det_array)
        
        # 4. Publish the visual feed so you can watch the drone's POV
        annotated_frame = results[0].plot()
        debug_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HumanDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
