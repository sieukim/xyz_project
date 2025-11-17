import os
import time
import cv2
import numpy as np
import pytesseract
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class YoloOcrNode(Node):
    def __init__(self):
        super().__init__("yolo_ocr_node")

        # ===== ROS2 파라미터 =====
        self.declare_parameter("model_path", "/home/sieun/Downloads/xyz_project/xyz_1st_project/doosan-robot2/dsr_example2/dsr_example/dsr_example/weights/Vehicle_number.pt")
        self.declare_parameter("camera_device", "/dev/video2")       # 또는 0
        self.declare_parameter("capture_interval_sec", 5.0)
        self.declare_parameter("lock_duration_sec", 5.0)
        self.declare_parameter("plate_class_id", -1)                  # -1이면 필터 안 함
        self.declare_parameter("min_box_w", 20)
        self.declare_parameter("min_box_h", 20)
        self.declare_parameter("conf_thres", 0.0)                     # 필요시 사용

        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.camera_device = self.get_parameter("camera_device").get_parameter_value().string_value
        self.CAPTURE_INTERVAL = float(self.get_parameter("capture_interval_sec").value)
        self.LOCK_DURATION = float(self.get_parameter("lock_duration_sec").value)
        self.PLATE_CLASS_ID = int(self.get_parameter("plate_class_id").value)
        self.MIN_W = int(self.get_parameter("min_box_w").value)
        self.MIN_H = int(self.get_parameter("min_box_h").value)
        self.CONF_THRES = float(self.get_parameter("conf_thres").value)

        # ===== 퍼블리셔 =====
        self.pub_ocr = self.create_publisher(String, "/ocr_text", 10)

        # ===== YOLO 모델 로드 =====
        self.model = YOLO(self.model_path)
        self.get_logger().info(f"YOLO model loaded: {self.model_path}")

        # ===== 카메라 =====
        self.cap = cv2.VideoCapture(self.camera_device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera: {self.camera_device}")
        else:
            self.get_logger().info(f"Camera opened: {self.camera_device}")

        # ===== 상태 변수 =====
        self.active_bbox = None       # (x1,y1,x2,y2)
        self.lock_until = 0.0
        self.last_capture_time = 0.0
        self.plate_text = ""
        os.makedirs("plates", exist_ok=True)

        # 메인 루프용 타이머 (프레임 레이트 제어: 30Hz 정도)
        self.timer = self.create_timer(1.0 / 30.0, self.loop)

    # ---------- 전처리 / OCR ----------
    def preprocess_ocr_roi(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        _, th = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        th_big = cv2.resize(th, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_LINEAR)
        return th_big

    def ocr_digits(self, img):
        cfg = "--psm 7 -c tessedit_char_whitelist=0123456789"
        txt = pytesseract.image_to_string(img, lang="eng", config=cfg)
        return txt.strip()

    # ---------- 유틸 ----------
    def _clamp_box(self, box, shape):
        x1, y1, x2, y2 = box
        h, w = shape[:2]
        x1 = max(0, min(x1, w - 1))
        x2 = max(0, min(x2, w))
        y1 = max(0, min(y1, h - 1))
        y2 = max(0, min(y2, h))
        return x1, y1, x2, y2

    def find_best_box(self, results):
        best_box, best_conf = None, 0.0
        if hasattr(results, "boxes") and results.boxes is not None:
            for box in results.boxes:
                xyxy = box.xyxy[0].cpu().numpy()
                x1, y1, x2, y2 = map(int, xyxy)
                conf = float(box.conf[0].cpu().item())
                cls_id = int(box.cls[0].cpu().item())

                if self.PLATE_CLASS_ID >= 0 and cls_id != self.PLATE_CLASS_ID:
                    continue
                if self.CONF_THRES > 0.0 and conf < self.CONF_THRES:
                    continue
                if (x2 - x1) < self.MIN_W or (y2 - y1) < self.MIN_H:
                    continue

                if conf > best_conf:
                    best_conf = conf
                    best_box = (x1, y1, x2, y2)
        return best_box

    # ---------- 메인 루프 ----------
    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        now = time.time()

        # 잠금 만료 시 갱신 가능
        if now >= self.lock_until:
            # 새 탐지
            results = self.model(frame, imgsz=640)[0]
            best_box = self.find_best_box(results)

            if best_box is not None:
                self.active_bbox = best_box
                self.lock_until = now + self.LOCK_DURATION

                # 즉시 캡처 & OCR
                x1, y1, x2, y2 = self._clamp_box(self.active_bbox, frame.shape)
                roi = frame[y1:y2, x1:x2]
                if roi.size != 0:
                    ts = int(now)
                    save_path = f"plates/captured_plate_{ts}.jpg"
                    cv2.imwrite(save_path, roi)
                    self.get_logger().info(f"saved: {save_path}")

                    th_big = self.preprocess_ocr_roi(roi)
                    self.plate_text = self.ocr_digits(th_big)
                    self.last_capture_time = now
                    self.get_logger().info(f"OCR: {self.plate_text if self.plate_text else '(empty)'}")

                    # 퍼블리시
                    if self.plate_text:
                        msg = String()
                        msg.data = self.plate_text
                        self.pub_ocr.publish(msg)
                        self.get_logger().info(f"OCR 텍스트 퍼블리시: {self.plate_text}")

                        # === 여기 추가: 한 번 퍼블리시 후 노드 종료 ===
                        self.get_logger().info("한 번 인식 완료, 노드 종료합니다.")
                        rclpy.shutdown()
                        return
            else:
                # 너무 자주 재시도 방지
                self.active_bbox = None
                self.lock_until = now + 1.0

        # ---- 시각화 ----
        if self.active_bbox is not None:
            x1, y1, x2, y2 = self._clamp_box(self.active_bbox, frame.shape)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            if self.plate_text:
                cv2.putText(frame, self.plate_text, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "No target", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        remain_lock = max(0, int(self.lock_until - now))
        remain_capture = (max(0, int(self.CAPTURE_INTERVAL - (now - self.last_capture_time)))
                          if self.last_capture_time > 0 else "init")
        cv2.putText(frame, f"Lock: {remain_lock}s | Next capture in: {remain_capture}s",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("YOLO + OCR (every 5s, ROS2)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    # 종료
    def destroy_node(self):
        try:
            if self.cap:
                self.cap.release()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = YoloOcrNode()
    try:
        rclpy.spin(node)
    finally:
        # 위에서 rclpy.shutdown()이 이미 호출되었으면 여기서는 ok()가 False라 다시 안 부름
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
