import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
from flask import Flask, jsonify
from flask_cors import CORS

# --- Flask & WebSocket 설정 ---
app = Flask(__name__)
CORS(app) # 모든 출처에서의 요청을 허용합니다.

# 마지막으로 수신된 OCR 텍스트를 저장할 변수
last_ocr_text = ""
last_ocr_text_lock = threading.Lock()

class OcrStreamerNode(Node):
    def __init__(self):
        super().__init__('ocr_streamer_node')
        
        # 1. /ocr_text 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/ocr_text',
            self.ocr_callback,
            10)
        self.get_logger().info('✅ OCR Streamer Node 시작...')
        self.get_logger().info('   -> /ocr_text 토픽 구독 중')

        # 2. Flask 서버를 8083 포트에서 실행
        self.flask_thread = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=8083),
            daemon=True)
        self.flask_thread.start()
        self.get_logger().info('🚀 HTTP 서버가 http://0.0.0.0:8083/ocr_text 에서 실행 대기 중...')

    def ocr_callback(self, msg: String):
        """/ocr_text 토픽 메시지를 받으면 변수에 저장"""
        global last_ocr_text
        with last_ocr_text_lock:
            last_ocr_text = msg.data
        self.get_logger().info(f'Received and stored OCR text: "{msg.data}"')

@app.route('/ocr_text')
def get_ocr_text():
    """저장된 최신 OCR 텍스트를 JSON 형태로 반환"""
    with last_ocr_text_lock:
        text_to_send = last_ocr_text
    return jsonify({"ocr_text": text_to_send})

def main(args=None):
    rclpy.init(args=args)
    node = OcrStreamerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()