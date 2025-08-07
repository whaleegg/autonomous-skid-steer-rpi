# bringup/launch/test_odometry.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')

    # 파라미터 파일 경로를 먼저 정의해야 함
    params_file = os.path.join(
        get_package_share_directory('params_package'),
        'config',
        'vehicle_params.yaml'
    )

    # 1. 하드웨어 드라이버 그룹 실행 (CAN 제외)
    #    - teleop.launch.py: joy_node, teleop_node 실행
    #    - imu.launch.py: imu_driver, imu_filter 실행
    #    - socketcan은 실제 하드웨어가 없으므로 실행하지 않음!
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'teleop.launch.py'))
    )
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'imu.launch.py'))
    )

    # 2. 오도메트리 그룹 실행 (변경 없음)
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'odometry.launch.py'))
    )

    # 핵심 제어 노드 실행
    core_controller_node = Node(
        package='controller_package',
        executable='core_controller_node',
        name='core_controller_node',
        output='screen',
        parameters=[params_file]
    )

    # 4. 가상 TC375 노드 실행 (핵심!)
    mock_tc375_node = Node(
        package='simulation_package',
        executable='mock_tc375_node',
        name='mock_tc375_node',
        output='screen'
    )

    return LaunchDescription([
        teleop_launch,
        imu_launch,
        odometry_launch,
        core_controller_node,
        mock_tc375_node,
    ])
