import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # imu_filter_madgwick를 위한 파라미터 파일 경로
    imu_filter_params_file = os.path.join(
        get_package_share_directory('params_package'),
        'config',
        'imu_filter.yaml'
    )
    
    # 1. IMU 드라이버 노드 (팀원이 만든 버전)
    imu_driver_node = Node(
        package='mpu9250driver', # 패키지 이름은 실제에 맞게 수정
        executable='mpu9250driver',
        name='imu_driver_node'
    )

    # 2. IMU 필터 노드
    imu_filter_node = Node(
        package='imu_tools', # imu_filter_madgwick은 imu_tools 패키지 안에 있음
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
        parameters=[imu_filter_params_file]
    )

    return LaunchDescription([
        imu_driver_node,
        imu_filter_node,
    ])
