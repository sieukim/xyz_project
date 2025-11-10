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

# --- Flask 앱 설정 (Realsense용) ---
flask_app_realsense = Flask(__name__) # 👈 앱 이름 변경 (충돌 방지)
CORS(flask_app_realsense) # 👈 앱에 CORS 지원 추가
frame_lock = threading.Lock()
latest_frame_bytes = None
placeholder_jpg_bytes = None
NODE_LOGGER = None

WIDTH = 640  # Realsense/YOLO 영상 크기
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

class RealsenseStreamerNode(Node):
    def __init__(self):
        super().__init__('realsense_streamer_node')
        global NODE_LOGGER
        NODE_LOGGER = self.get_logger()                   

        self.bridge = CvBridge()
        _create_placeholder_image("REALSENSE - WAITING...")

        # fuel_task_manager가 발행하는 /fuel/image_result 토픽을 구독
        self.subscription = self.create_subscription(
            Image,
            '/fuel/realsense_color', # 👈 realsense_manager_ros가 발행하는 토픽
            self.image_callback,
            qos_profile_sensor_data) # QoS 프로파일 사용
        self.get_logger().info('✅ Realsense 스트리머 노드 시작...')
        self.get_logger().info('   -> /fuel/realsense_color 토픽 구독 중')

        # Flask 서버를 8081 포트에서 실행
        self.flask_thread = threading.Thread(
            # 👈 앱 이름 변경
            target=lambda: flask_app_realsense.run(host='0.0.0.0', port=8081, threaded=True, debug=False),
            daemon=True)
        self.flask_thread.start()
        self.get_logger().info('🚀 Flask 서버가 http://0.0.0.0:8081/ 에서 실행 대기 중...')

    def image_callback(self, msg: Image):
        """ROS 2 이미지 토픽을 JPEG 바이트로 변환하여 저장"""
        global latest_frame_bytes
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            (flag, encoded_image) = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not flag:
                self.get_logger().warn("Realsense 프레임 JPEG 인코딩 실패", throttle_duration_sec=5)
                return
            with frame_lock:
                latest_frame_bytes = bytearray(encoded_image)
        except Exception as e:
            self.get_logger().error(f'Realsense 이미지 변환 실패: {e}')

# --- Flask 라우트 (8081 포트) ---

def generate_realsense_stream():
    """Realsense 비디오 스트림을 생성하는 제너레이터 함수"""
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
        
        # Realsense는 30FPS로 가정, 필요시 조절
        time.sleep(1.0 / 30) 

@flask_app_realsense.route('/video_feed') # 👈 앱 이름 변경
def video_feed():
    """MJPEG 비디오 스트림을 제공하는 Flask 라우트"""
    client_ip = request.remote_addr
    print(f"[INFO] Flask (8081): 🎬 Realsense 새 클라이언트 연결됨: {client_ip}")
    return Response(generate_realsense_stream(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@flask_app_realsense.route('/') # 👈 앱 이름 변경
def index():
    """✅ WebView가 로드할 기본 HTML 페이지 (8081 포트용)"""
    client_ip = request.remote_addr
    print(f"[INFO] Flask (8081): 🖥️ WebView 클라이언트 연결됨 (/): {client_ip}")
    html = f"""
    <html>
        <head><title>Realsense Stream</title>
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
    streamer_node = None
    try:
        streamer_node = RealsenseStreamerNode()
        rclpy.spin(streamer_node)
    except KeyboardInterrupt:
        print("Realsense Streamer: KeyboardInterrupt 감지, 종료 중...")
    finally:
        if streamer_node:
            streamer_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Realsense Streamer: 종료 완료.")

if __name__ == '__main__':
    main()