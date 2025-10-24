import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraStateSubscriber(Node):
    def __init__(self):
        super().__init__('camera_state_subscriber')
        self.subscription = self.create_subscription(
            String,
            'camera_state',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        if msg.data == 'ON':
            self.get_logger().info('✅ 카메라가 켜졌습니다!')
        elif msg.data == 'OFF':
            self.get_logger().info('❌ 카메라가 꺼졌습니다!')
        else:
            self.get_logger().warn(f'⚠️ 알 수 없는 상태: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraStateSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
