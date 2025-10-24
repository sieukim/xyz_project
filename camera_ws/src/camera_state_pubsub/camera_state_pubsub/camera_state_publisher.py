import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import time

class CameraStatePublisher(Node):
    def __init__(self):
        super().__init__('camera_state_publisher')
        # 파라미터
        self.declare_parameter('device_index', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('open_retry', 3)
        self.declare_parameter('window_name', 'Camera Preview')
        self.declare_parameter('frame_period_sec', 0.1)  # ≈ 20 FPS(깜빡임 완화)

        # 퍼블리셔
        self.state_pub = self.create_publisher(String, 'camera_state', 10)

        self.cap = None
        self.window_name = self.get_parameter('window_name').get_parameter_value().string_value

        # 카메라 열기
        self.cap = self.open_camera()
        if self.cap is None:
            self.publish_state('ERROR')
            raise RuntimeError('카메라를 열 수 없습니다.')

        # 창 생성(한 번만)
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)

        # 상태 발행: ON
        self.publish_state('ON')
        self.get_logger().info('📸 카메라 항상 ON 모드 시작')

        # 프레임 타이머
        period = float(self.get_parameter('frame_period_sec').get_parameter_value().double_value)
        self.frame_timer = self.create_timer(period, self.frame_callback)

    def publish_state(self, s: str):
        msg = String()
        msg.data = s
        self.state_pub.publish(msg)

    def open_camera(self):
        idx = self.get_parameter('device_index').get_parameter_value().integer_value
        w   = self.get_parameter('width').get_parameter_value().integer_value
        h   = self.get_parameter('height').get_parameter_value().integer_value
        retries = self.get_parameter('open_retry').get_parameter_value().integer_value

        for attempt in range(1, retries + 1):
            cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                ok, _ = cap.read()
                if ok:
                    return cap
                cap.release()
            self.get_logger().warn(f'카메라 열기 재시도 {attempt}/{retries} 실패…')
            time.sleep(0.4)
        return None

    def frame_callback(self):
        if self.cap is None:
            return
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('프레임 읽기 실패…')
            return
        cv2.imshow(self.window_name, frame)
        # 'q'로 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('사용자 종료(q)')
            self.shutdown_node()

    def shutdown_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
        except:
            pass
        self.publish_state('OFF')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt')
    finally:
        node.shutdown_node()

if __name__ == '__main__':
    main()


#video2 depth camera