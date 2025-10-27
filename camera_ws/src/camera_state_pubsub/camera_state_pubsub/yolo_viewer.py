import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from datetime import datetime


CONFIDENCE_THRESHOLD = 0.7
CENTER_TOLERANCE_PX = 10

# 네가 만든 annotate_with_detection()을 같은 패키지에서 import 하거나
# 여기 파일에 붙여넣어도 됨
def annotate_with_detection(frame, yolo_result):
    h, w = frame.shape[:2]
    target_x, target_y = w // 2, h // 2  # 프레임 중심
    annotated = yolo_result.plot()

    # 프레임 중심 마커(흰색 X)
    cv2.drawMarker(
        annotated, (target_x, target_y), (255, 255, 255),
        markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2
    )

    info = {
        "detected": False,
        "center_x": None,
        "center_y": None,
        "dx_pixels": None,
        "dy_pixels": None,
        "centered": False,
        "conf": None,
    }

    boxes = getattr(yolo_result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        cv2.putText(annotated, "No detection", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (50, 50, 255), 2)
        return annotated, info

    # 신뢰도 높은 박스 하나 선택
    best_idx, best_conf = None, -1.0
    for i in range(len(boxes)):
        conf = float(boxes.conf[i].item()) if hasattr(boxes, "conf") else 0.0
        if conf >= CONFIDENCE_THRESHOLD and conf > best_conf:
            best_idx, best_conf = i, conf

    if best_idx is None:
        cv2.putText(annotated, "Low-confidence detections only", (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 140, 255), 2)
        return annotated, info

    # 바운딩 박스 중심 계산
    x1, y1, x2, y2 = map(int, boxes.xyxy[best_idx].tolist())
    x_center = (x1 + x2) // 2
    y_center = (y1 + y2) // 2

    # 빨간 점(객체 중심)
    cv2.circle(annotated, (x_center, y_center), 6, (0, 0, 255), -1)

    # 프레임 중심과 오차
    dx_pixels = (target_x - x_center)
    dy_pixels = (target_y - y_center)

    # 텍스트
    cv2.putText(annotated, f"center=({x_center},{y_center}) conf={best_conf:.2f}",
                (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 200), 2)
    cv2.putText(annotated, f"delta_px=({dx_pixels},{dy_pixels})",
                (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (200, 255, 200), 2)

    centered = (abs(dx_pixels) < CENTER_TOLERANCE_PX and abs(dy_pixels) < CENTER_TOLERANCE_PX)
    if centered:
        cv2.putText(annotated, "Centered!", (20, 170),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)

    info.update({
        "detected": True,
        "center_x": x_center,
        "center_y": y_center,
        "dx_pixels": dx_pixels,
        "dy_pixels": dy_pixels,
        "centered": centered,
        "conf": best_conf,
    })
    return annotated, info


class YOLOViewer(Node):
    def __init__(self):
        super().__init__('yolo_viewer')
        self.declare_parameter('model_path', '/home/shim/ws/camera_ws/best.pt')  
                                            #/home/shim/github/yolo_dectect_traing/yolov8n.pt 
        self.declare_parameter('window', False)  # True면 OpenCV 창 표시

        self.bridge = CvBridge()
        mp = self.get_parameter('model_path').get_parameter_value().string_value
        self.show_window = self.get_parameter('window').get_parameter_value().bool_value

        self.model = YOLO(mp)
        self.get_logger().info(f'Loaded YOLO model: {mp}')

        # 구독: 카메라 원본
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.cb, qos_profile_sensor_data
        )

        # 퍼블리시: 주석(annotated) 이미지
        self.pub_anno = self.create_publisher(Image, '/camera/image_annotated', qos_profile_sensor_data)

        if self.show_window:
            cv2.namedWindow('YOLO Annotated', cv2.WINDOW_NORMAL)

        self.pub_class = self.create_publisher(String, '/detected_object', 10)
        self.get_logger().info('YOLOViewer: 객체 인식 결과를 /detected_object 로 퍼블리시합니다.')

    def cb(self, msg: Image):
        # ROS Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO 추론
        results = self.model.predict(source=frame, device='cpu', imgsz=640, conf=0.4, verbose=False)
        annotated, info = annotate_with_detection(frame, results[0])

        boxes = getattr(results[0], "boxes", None)
        if boxes is not None and len(boxes) > 0:
            names = self.model.names  # {0:'green_car', 1:'yellow_car', 2:'orange_car', ...}
            seen_classes = set()
            for i in range(len(boxes)):
                cls_id = int(boxes.cls[i].item()) if hasattr(boxes, "cls") else -1
                name = names.get(cls_id, f'class_{cls_id}')
                if name in {'green_car', 'yellow_car', 'orange_car'}:
                    seen_classes.add(name)
            if seen_classes:
                msg_out = String()
                msg_out.data = ','.join(seen_classes)
                self.pub_class.publish(msg_out)
                self.get_logger().info(f"🎯 인식됨: {msg_out.data}")

        # 화면 표시
        if self.show_window:
            cv2.imshow('YOLO Annotated', annotated)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
                return

        # 주석 이미지를 퍼블리시해서 RViz에서 보게 하기
        anno_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        anno_msg.header = msg.header
        self.pub_anno.publish(anno_msg)

def main():
    rclpy.init()
    node = YOLOViewer()
    try:
        rclpy.spin(node)
    finally:
        if node.get_parameter('window').get_parameter_value().bool_value:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
