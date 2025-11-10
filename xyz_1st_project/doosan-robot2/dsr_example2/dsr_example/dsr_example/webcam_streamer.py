import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from flask import Flask, Response, request 
from flask_cors import CORS # 👈 CORS 라이브러리 임포트
import time
import numpy as np
from rclpy.qos import qos_profile_sensor_data

# --- Flask 앱 설정 (Webcam용) ---
flask_app_webcam = Flask(__name__)
CORS(flask_app_webcam) # 👈 앱에 CORS 지원 추가
frame_lock = threading.Lock()
latest_frame_bytes = None
placeholder_jpg_bytes = None
NODE_LOGGER = None

WIDTH = 640
HEIGHT = 480

def _create_placeholder_image(text: str) -> bytes:
    """플레이스홀더 이미지를 생성하고 JPEG 바이트로 반환합니다."""
    global placeholder_jpg_bytes
    try:
        img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        font = cv2.FONT_HERSHEY_SIMPLEX
        (text_width, text_height), _ = cv2.getTextSize(text, font, 1, 2)
        text_x = (WIDTH - text_width) // 2
        text_y = (HEIGHT + text_height) // 2
        cv2.putText(img, text, (text_x, text_y), font, 1, (255, 255, 255), 2)
        
        (flag, encoded_image) = cv2.imencode(".jpg", img)
        if flag:
            placeholder_jpg_bytes = bytearray(encoded_image)
            if NODE_LOGGER:
                NODE_LOGGER.info(f"📸 '{text}' 플레이스홀더 이미지 생성됨.")
        return placeholder_jpg_bytes
    except Exception as e:
        if NODE_LOGGER:
            NODE_LOGGER.error(f"플레이스홀더 이미지 생성 실패: {e}")
    return b''

class WebcamStreamerNode(Node):
    def __init__(self):
        super().__init__('webcam_streamer_node')
        global NODE_LOGGER
        NODE_LOGGER = self.get_logger()

        self.bridge = CvBridge()
        _create_placeholder_image("WEBCAM - WAITING...")

        # webcam_manager_ros가 발행하는 /webcam/image_raw 토픽을 구독
        self.subscription = self.create_subscription(
            Image,
            '/fuel/webcam_color', # 👈 webcam_manager_ros가 발행하는 토픽
            self.image_callback,
            qos_profile_sensor_data)
        self.get_logger().info('✅ Webcam 스트리머 노드 시작...')
        self.get_logger().info('   -> /fuel/webcam_color 토픽 구독 중')

        # Flask 서버를 8082 포트에서 실행 (포트 충돌 방지)
        self.flask_thread = threading.Thread(
            target=lambda: flask_app_webcam.run(host='0.0.0.0', port=8082, threaded=True, debug=False),
            daemon=True)
        self.flask_thread.start()
        self.get_logger().info('🚀 Flask 서버가 http://0.0.0.0:8082/ 에서 실행 대기 중...')

    def image_callback(self, msg: Image):
        global latest_frame_bytes
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            (flag, encoded_image) = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not flag:
                self.get_logger().warn("Webcam 프레임 JPEG 인코딩 실패", throttle_duration_sec=5)
                return
            with frame_lock:
                latest_frame_bytes = bytearray(encoded_image)
        except Exception as e:
            self.get_logger().error(f'Webcam 이미지 변환 실패: {e}')

def generate_webcam_stream():
    global latest_frame_bytes, placeholder_jpg_bytes
    while True:
        frame_to_send = None
        with frame_lock:
            if latest_frame_bytes:
                frame_to_send = latest_frame_bytes
            else:
                frame_to_send = placeholder_jpg_bytes
        if frame_to_send:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_to_send + b'\r\n')
        else:
            time.sleep(0.1)
        time.sleep(1.0 / 30)

@flask_app_webcam.route('/video_feed')
def video_feed():
    return Response(generate_webcam_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@flask_app_webcam.route('/')
def index():
    """✅ WebView가 로드할 기본 HTML 페이지 (8082 포트용)"""
    client_ip = request.remote_addr
    print(f"[INFO] Flask (8082): 🖥️ WebView 클라이언트 연결됨 (/): {client_ip}")
    html = f"""
    <html>
        <head><title>Webcam Stream</title>
            <style>
                body {{ margin: 0; padding: 0; background-color: #000; }}
                img {{ width: 100%; height: 100%; object-fit: contain; }}
            </style>
        </head>
        <body>
            <img src="/video_feed" />
        </body>
    </html>
    """
    return Response(html, mimetype='text/html')

def main(args=None):
    rclpy.init(args=args)
    streamer_node = WebcamStreamerNode()
    rclpy.spin(streamer_node)
    streamer_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()