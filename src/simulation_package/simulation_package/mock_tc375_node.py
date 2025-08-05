import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from std_msgs.msg import Int64 # RPM 발행용 (선택적)
import struct

class MockTC375Node(Node):
    def __init__(self):
        super().__init__('mock_tc375_node')

        # RPi -> TC375 제어 명령 수신
        self.subscription = self.create_subscription(
            Frame,
            '/to_can_bus',
            self.to_can_bus_callback,
            10)
        
        # TC375 -> RPi 센서 데이터 발행
        self.publisher = self.create_publisher(Frame, '/from_can_bus', 10)

        # (디버깅용) RPM 토픽 발행기 (선택적)
        self.left_rpm_pub = self.create_publisher(Int64, '/left_wheel_rpm_mock', 10)
        self.right_rpm_pub = self.create_publisher(Int64, '/right_wheel_rpm_mock', 10)

        # 33ms (약 30Hz) 마다 send_feedback 함수 호출
        self.timer = self.create_timer(0.033, self.send_feedback)

        # === 가상 차량 모델 파라미터 ===
        # PWM 1당 RPM 변환 계수 (튜닝 필요)
        self.PWM_TO_RPM_RATIO = 1.0 # 255 / 255 (예: 100 PWM -> 160 RPM)
        
        # 현재 차량의 바퀴 RPM 상태
        self.current_left_rpm = 0
        self.current_right_rpm = 0

        self.get_logger().info("Mock TC375 Node has been started.")

    def to_can_bus_callback(self, msg: Frame):
        # 제어 명령(ID 0x100)만 처리
        if msg.id == 0x100:
            left_dir = msg.data[0]
            left_pwm = msg.data[1]
            right_dir = msg.data[2]
            right_pwm = msg.data[3]

            # 수신된 PWM 값을 RPM으로 변환하여 상태 업데이트
            # (dir=0 이면 역방향, -)
            self.current_left_rpm = left_pwm * self.PWM_TO_RPM_RATIO * (1 if left_dir == 1 else -1)
            self.current_right_rpm = right_pwm * self.PWM_TO_RPM_RATIO * (1 if right_dir == 1 else -1)

            # (디버깅용) RPM 토픽 발행
            self.left_rpm_pub.publish(Int64(data=int(self.current_left_rpm)))
            self.right_rpm_pub.publish(Int64(data=int(self.current_right_rpm)))

    def send_feedback(self):
        # 현재 RPM 상태를 CAN 메시지로 패킹
        frame = Frame()
        frame.header.stamp = self.get_clock().now().to_msg()
        frame.id = 0x201 # 오도메트리 피드백 ID
        frame.dlc = 8
        
        # float가 아닌 int32로 RPM 전송 (Confluence 문서 기준)
        left_rpm_bytes = struct.pack('<i', int(self.current_left_rpm))
        right_rpm_bytes = struct.pack('<i', int(self.current_right_rpm))
        
        frame.data = list(left_rpm_bytes + right_rpm_bytes)
        
        # /from_can_bus 토픽으로 발행
        self.publisher.publish(frame)

def main(args=None):
    rclpy.init(args=args)
    node = MockTC375Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
