import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # bringup 패키지의 공유 디렉토리 경로를 미리 변수에 저장
    bringup_dir = get_package_share_directory('bringup')
    # params 패키지의 공유 디렉토리 경로
    params_dir = get_package_share_directory('params_package')

    # 사용할 파라미터 파일의 전체 경로
    vehicle_params_file = os.path.join(params_dir, 'config', 'vehicle_params.yaml')

    # === 1. 기능 그룹별 Launch 파일 포함 ===

    # a. 하드웨어 드라이버 그룹 실행 (joy, teleop, imu, socketcan)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'hardware.launch.py')
        )
    )

    # b. 오도메트리/로컬라이제이션 그룹 실행 (ekf_node)
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'odometry.launch.py')
        )
    )

    # === 2. 핵심 로직 노드 실행 ===

    # a. core_controller_node (상태 관리 및 제어)
    core_controller_node = Node(
        package='controller_package',
        executable='core_controller_node',
        name='core_controller_node',
        output='screen',
        parameters=[vehicle_params_file]
    )

    # 4. 자율 기능 그룹 실행
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'autonomy.launch.py')
        )
    )

    # === 3. 최종 실행 목록 구성 ===
    return LaunchDescription([
        hardware_launch,
        odometry_launch,
        core_controller_node,
        autonomy_launch,
    ])
