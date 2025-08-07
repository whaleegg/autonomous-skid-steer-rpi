import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # params 패키지의 공유 디렉토리 경로
    params_dir = get_package_share_directory('params_package')

    # imu_filter_madgwick를 위한 파라미터 파일 경로
    imu_filter_params_file = os.path.join(params_dir, 'config', 'imu_filter.yaml')

    # 1. IMU 드라이버 노드 (팀원이 만든 버전)
    imu_driver_node = Node(
        package='mpu9250driver', # 패키지 이름은 실제에 맞게 수정
        executable='mpu9250driver',
        name='imu_driver_node',
	#remappings=[
        #    ('imu', '/imu/data_raw')
        #]
    )

    # 2. IMU 필터 노드
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_node',
	output='screen',
        #parameters=[imu_filter_params_file]
	parameters=[{
            'use_mag': True,
            'publish_tf': False, # <--- 여기서 직접 False를 지정
            'remove_gravity_vector': True
        }]
        # 입력 토픽은 기본값(/imu/data_raw, /imu/mag)을 사용하므로 리매핑 불필요
    )

    return LaunchDescription([
        imu_driver_node,
        imu_filter_node,
    ])
