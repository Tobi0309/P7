import os
import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose 
from std_srvs.srv import Trigger




class YOLO_Human_Detection(Node):
    def __init__(self):
        super().__init__('yolo_human_detection')
        forestpersons_model = os.path.join(
            get_package_share_directory('P7_human_detection'), 'pre_trained_yolo_model', 'best.pt')

        self.declare_parameter('model_path', forestpersons_model)
        self.declare_parameter('input_topic', '/UAV/rgbd_camera/color/image_raw/compressed')
        self.declare_parameter('output_topic', '/yolo/image_annotated')
        self.declare_parameter('detections_topic', '/yolo/bounding_boxes')
        self.declare_parameter('conf', 0.25) #Confidence threshold
        self.declare_parameter('imgsz', 640) #Image size for inference
        self.declare_parameter('device', '0')   # '0' for GPU, 'cpu' for CPU #WE USE GPU
        self.declare_parameter('min_detections', 10) #Minimum number of consecutive detections to trigger an action
        self.detectionCount = 0  #Counter for consecutive detections
        self.stop_flag = False #Flag to indicate if the drone should stop


        
        #Store values of parameters in this node
        self.conf = self.get_parameter('conf').value
        self.imgsz = self.get_parameter('imgsz').value
        self.device = self.get_parameter('device').value
        self.min_detections = self.get_parameter('min_detections').value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        self.bridge = CvBridge()
        self.model = YOLO(model_path)
        self.get_logger().info(f'Loaded {model_path}, classes: {self.model.names}')

        # Create subscriber for realsense data. 
        self.sub = self.create_subscription(
            CompressedImage,
            self.get_parameter('input_topic').value,
            self._working_callback,
            qos_profile_sensor_data)

        #Create publisher for annotated images
        self.pub = self.create_publisher(
            Image, self.get_parameter('output_topic').value, 10)
        
        #Create publisher for bounding boxes
        self.det_pub = self.create_publisher(
            Detection2DArray, self.get_parameter('detections_topic').value, 10)

        #Create a service to stop the drone when a human is detected
        self.stop_srv = self.create_service(
            Trigger, '/drone/stop', self._stop_service_callback)



    def _working_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = np.rot90(frame, 2).copy()
        result = self.model.predict(
            frame, conf=self.conf, imgsz=self.imgsz,
            device=self.device, verbose=False)[0]   
        annotated_frame = result.plot()

        #Create a new image message
        out = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        out.header = msg.header

        #Publish the annotated image message
        self.pub.publish(out)

        # Create bounding box message
        det_array = Detection2DArray()
        det_array.header = msg.header
        for box in result.boxes:
            # xywh = center x, center y, width, height (in pixels)
            cx, cy, w, h = box.xywh[0].tolist()
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])

            det = Detection2D()
            det.header = msg.header
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = self.model.names[cls_id]
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            det_array.detections.append(det)

        #Publish the bounding box message
        self.det_pub.publish(det_array)



        #Log the number of detections
        n = len(result.boxes)
        self.get_logger().info(f'{n} detection(s)', throttle_duration_sec=2.0)

        if n > 0:
            self.detectionCount += 1
        else:
            self.detectionCount = 0

        if self.detectionCount >= self.min_detections:
            self.get_logger().info('Human detected. Stopping the drone.', throttle_duration_sec=2.0)
            self.stop_flag = True

    def _stop_service_callback(self, request, response):
        response.success = self.stop_flag #bool der bliver true hvis vi har set menneske i 10 frames

        if self.stop_flag:
            response.message = 'Human detected. Stopping the drone.'    
        else:
            response.message = 'No human detected. Drone can continue flying.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YOLO_Human_Detection()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
