import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import yaml

def generate_launch_description():
    
    # --- 1. 파라미터 파일들의 경로를 동적으로 찾기 ---
    params_pkg_share_dir = get_package_share_directory('params_package')
    
    params_file_path = os.path.join(
        params_pkg_share_dir,
        'config',
        'vehicle_params.yaml'
    )
    
    calibration_file_path = os.path.join(
        params_pkg_share_dir,
        'config',
        'joystick_cal.yaml'
    )

    # --- 2. YAML 파일을 읽어 파라미터 딕셔너리 생성 및 병합 ---
    try:
        with open(params_file_path, 'r') as f:
            # vehicle_params.yaml 파일에서 teleop_node 섹션만 읽어옴
            teleop_params = yaml.safe_load(f)['teleop_node']['ros__parameters']
    except (FileNotFoundError, KeyError):
        # 파일이 없거나 섹션이 없으면 비어있는 딕셔너리로 시작
        teleop_params = {}
        
    # calibration_file 파라미터의 값을 동적으로 찾은 경로로 덮어쓰기
    teleop_params['calibration_file'] = calibration_file_path

    # --- 3. 노드들 선언 ---
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device file'
    )
    
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': LaunchConfiguration('joy_dev'), 'deadzone': 0.1}]
    )

    teleop_node = Node(
        package='teleop_package',
        executable='teleop_node',
        name='teleop_node',
        output='screen',
        # --- 4. 최종적으로 합쳐진 파라미터 딕셔너리 하나만 전달 ---
        parameters=[teleop_params] 
    )

    # --- 5. 최종 LaunchDescription 생성 ---
    return LaunchDescription([
        joy_dev_arg,
        joy_node,
        teleop_node,
    ])
