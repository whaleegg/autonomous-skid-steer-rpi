# ROS 2 Launch 시스템의 필수 라이브러리들을 import 합니다.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # --- Launch Argument 선언 ---
    # 'joy_dev' 라는 이름의 launch argument를 선언합니다.
    # 이 argument를 통해 실행 시 조이스틱 장치 경로를 바꿀 수 있습니다.
    # 기본값은 '/dev/input/js0' 입니다.
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )

    # --- 노드 선언 ---
    # 1. joy_node 실행 설정
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': LaunchConfiguration('joy_dev'), # 위에서 선언한 argument 값을 'dev' 파라미터로 전달
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    # 2. teleop_node 실행 설정
    teleop_node = Node(
        package='teleop_package',
        executable='teleop_node',
        name='teleop_node',
        output='screen', # 노드의 로그(INFO, WARN, ERROR 등)를 터미널 화면에 바로 출력
        # 노드가 사용하는 파라미터를 YAML 파일에서 불러오도록 설정 (향후 확장)
        # parameters=[params_file] 
    )

    # --- 최종 LaunchDescription 생성 ---
    # 실행할 모든 argument와 노드들을 리스트에 담아 반환합니다.
    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        teleop_node,
    ])
