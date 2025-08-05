from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import yaml
import os
import time

class JoystickCalibrationNode(Node):
    def __init__(self):
        super().__init__('joystick_calibration_node')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        self.axes_data = {}
        self.state = 'INITIALIZING' # 초기 상태 추가
        
        # 1. 파일 저장 경로를 생성자에서 결정
        try:
            self.save_dir_ = "/root/ros_ws/src/params_package/config"
            self.calibration_file_path_ = os.path.join(self.save_dir_, "joystick_cal.yaml")

            self.state = 'CALIBRATING_CENTER'
            self.start_time = self.get_clock().now()
            self.get_logger().info("--- 조이스틱 캘리브레이션 시작 ---")
            self.get_logger().info("1. [중앙값 측정] 5초간 조이스틱을 가만히 두세요...")
        except Exception as e:
            self.get_logger().error(f"params_package를 찾을 수 없습니다. 먼저 빌드해주세요: {e}")
            self.state = 'ERROR'

    def joy_callback(self, msg: Joy):
        # 모든 축에 대한 데이터 구조를 미리 생성
        for i in range(len(msg.axes)):
            if i not in self.axes_data:
                self.axes_data[i] = {'center': 0.0, 'deadzone_min': 0.0, 'deadzone_max': 0.0, 'min': 0.0, 'max': 0.0}

        if self.state == 'CALIBRATING_CENTER':
            for i, value in enumerate(msg.axes):
                self.axes_data[i]['center'] = round(value, 4)
        
        elif self.state == 'CALIBRATING_RANGE':
            for i, value in enumerate(msg.axes):
                # 데드존 경계값 업데이트
                # 중앙값보다 크면서, 기록된 deadzone_max보다 작으면 업데이트
                if value > self.axes_data[i]['center'] and (self.axes_data[i]['deadzone_max'] == 0.0 or value < self.axes_data[i]['deadzone_max']):
                    self.axes_data[i]['deadzone_max'] = round(value, 4)
                # 중앙값보다 작으면서, 기록된 deadzone_min보다 크면 업데이트
                if value < self.axes_data[i]['center'] and (self.axes_data[i]['deadzone_min'] == 0.0 or value > self.axes_data[i]['deadzone_min']):
                    self.axes_data[i]['deadzone_min'] = round(value, 4)

                # 최소/최대값 업데이트
                if self.axes_data[i]['min'] == 0.0 or value < self.axes_data[i]['min']:
                    self.axes_data[i]['min'] = round(value, 4)
                if self.axes_data[i]['max'] == 0.0 or value > self.axes_data[i]['max']:
                    self.axes_data[i]['max'] = round(value, 4)

    def transition_state(self):
        now = self.get_clock().now()
        elapsed_time = (now - self.start_time).nanoseconds / 1e9

        if self.state == 'CALIBRATING_CENTER' and elapsed_time > 5.0:
            self.get_logger().info("중앙값 기록 완료.")
            self.get_logger().info("-----------------------------------------------------")
            self.get_logger().info("2. [범위 측정] 20초간 모든 스틱을 '아주 천천히' 중앙에서 끝까지, 여러 번 움직여주세요.")
            self.get_logger().info("   (특히 중앙에서 살짝 움직이는 동작이 중요합니다!)")
            self.state = 'CALIBRATING_RANGE'
            self.start_time = now
        
        elif self.state == 'CALIBRATING_RANGE' and elapsed_time > 20.0:
            self.save_calibration()
            self.get_logger().info("캘리브레이션 완료. 노드를 종료합니다.")
            self.state = 'DONE'
            rclpy.shutdown()

    def save_calibration(self):
        if self.state == 'ERROR': return

        # 2. calibration_data 변수를 여기서 정의
        calibration_data = {'axes': dict(sorted(self.axes_data.items()))}
        
        try:
            # 3. 디렉토리가 없으면 생성 (경로는 생성자에서 설정한 멤버 변수 사용)
            os.makedirs(self.save_dir_, exist_ok=True)
            
            with open(self.calibration_file_path_, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False, sort_keys=False)
            
            self.get_logger().info(f"캘리브레이션 데이터가 '{self.calibration_file_path_}' 파일에 저장되었습니다.")
            print("\n--- 저장된 캘리브레이션 데이터 ---")
            print(yaml.dump(calibration_data, default_flow_style=False, sort_keys=False))
            print("---------------------------------")
        except Exception as e:
            self.get_logger().error(f"파일 저장 실패: {e}")


def main(args=None):
    rclpy.init(args=args)
    calibration_node = JoystickCalibrationNode()
    
    if calibration_node.state != 'ERROR':
        while rclpy.ok():
            rclpy.spin_once(calibration_node, timeout_sec=0.05)
            calibration_node.transition_state()
            if calibration_node.state == 'DONE':
                break
    
    calibration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
