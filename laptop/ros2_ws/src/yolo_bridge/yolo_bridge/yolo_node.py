import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import torch

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    ObjectHypothesisWithPose,
    BoundingBox2D,
    Pose2D,
    Point2D,
)


class YoloBridgeNode(Node):
    def __init__(self):
        super().__init__('yolo_bridge')

        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/inference/image_raw')
        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('model_name', 'yolov5n')

        self.declare_parameter('inference_size', 256)
        self.declare_parameter('frame_skip', 1)

        self.declare_parameter('class_id_mode', 'index')
        self.declare_parameter('publish_debug', True)

        input_topic      = self.get_parameter('input_topic').value
        output_topic     = self.get_parameter('output_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        model_name       = self.get_parameter('model_name').value

        self.get_logger().info(f'Input topic        : {input_topic}')
        self.get_logger().info(f'Output image topic : {output_topic}')
        self.get_logger().info(f'Detections topic   : {detections_topic}')
        self.get_logger().info(f'Loading YOLOv5 model: {model_name}')

        
        self.model = torch.hub.load(
                                    '/absolute/path/to/yolov5',
                                    'custom',
                                    path='../../../../../weight/yolov5n.pt',
                                    source='local'
                                )

        self.model.to('cpu')
        self.model.eval()

        self.model.conf = 0.4
        self.model.iou = 0.5
        self.model.max_det = 5

        self.bridge = CvBridge()
        self.frame_count = 0

        self.sub = self.create_subscription(Image, input_topic, self.image_callback, 10)
        self.pub_image = self.create_publisher(Image, output_topic, 10)
        self.pub_detections = self.create_publisher(Detection2DArray, detections_topic, 10)

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge 변환 실패: {e}')
            return

        self.frame_count += 1
        frame_skip = int(self.get_parameter('frame_skip').value)

        publish_debug = bool(self.get_parameter('publish_debug').value)
        class_id_mode = str(self.get_parameter('class_id_mode').value).strip().lower()

        if frame_skip > 0 and (self.frame_count % (frame_skip + 1) != 0):
            if publish_debug:
                out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
                out_msg.header = msg.header
                self.pub_image.publish(out_msg)
            return

        img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        inference_size = int(self.get_parameter('inference_size').value)

        with torch.no_grad():
            results = self.model(img_rgb, size=inference_size)

        det_array_msg = Detection2DArray()
        det_array_msg.header = msg.header

        for det in results.xyxy[0].tolist():
            x1, y1, x2, y2, conf, cls = det
            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
            conf = float(conf)
            cls = int(cls)

            label_name = self.model.names[cls]

            if publish_debug:
                label = f"{label_name} {conf:.2f}"
                cv2.rectangle(cv_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_img, label, (x1, max(y1 - 5, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 1, cv2.LINE_AA)

            det_msg = Detection2D()
            det_msg.header = msg.header

            hyp = ObjectHypothesisWithPose()

            if class_id_mode == 'index':
                hyp.hypothesis.class_id = str(cls)      
            else:
                hyp.hypothesis.class_id = label_name    

            hyp.hypothesis.score = conf
            det_msg.results.append(hyp)

            bbox = BoundingBox2D()
            center = Pose2D()
            center.position = Point2D()
            center.position.x = float((x1 + x2) / 2.0)
            center.position.y = float((y1 + y2) / 2.0)
            center.theta = 0.0

            bbox.center = center
            bbox.size_x = float(x2 - x1)
            bbox.size_y = float(y2 - y1)
            det_msg.bbox = bbox

            det_array_msg.detections.append(det_msg)

        if publish_debug:
            out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
            out_msg.header = msg.header
            self.pub_image.publish(out_msg)

        self.pub_detections.publish(det_array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
