# bringup/launch/skid_steer_robot.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 파라미터 파일들의 경로를 찾음
    params_file = os.path.join(
        get_package_share_directory('params_package'),
        'config',
        'vehicle_params.yaml'
    )

    ekf_params_file = os.path.join(
        get_package_share_directory('params_package'),
        'config',
        'ekf.yaml'
    )

    # 2. 각 기능 그룹의 Launch 파일을 '포함(Include)'하도록 설정
    
    # teleop.launch.py를 포함 (joy_node, teleop_node 실행)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bringup'), 'launch', 'teleop.launch.py')
        )
        # 여기에 joy_dev:='/dev/input/js1' 처럼 파라미터를 넘겨줄 수도 있음
    )

    # ros2_socketcan의 sender/receiver 노드를 실행하는 Launch 파일을 포함
    socketcan_sender_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_sender.launch.py')
        ),
        launch_arguments={'interface': 'can0'}.items()
    )

    socketcan_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_receiver.launch.py')
        ),
        launch_arguments={'interface': 'can0'}.items()
    )

    # c. imu.launch.py (imu_driver + imu_filter 실행) - 새로 만듦
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bringup'), 'launch', 'imu.launch.py')
        )
    )


    # 3. 개별 핵심 노드들을 직접 실행하도록 설정

    # a. core_controller_node (상태 관리, CAN <-> ROS 변환, /odom/wheel 발행)
    core_controller_node = Node(
        package='controller_package',
        executable='core_controller_node',
        name='core_controller_node',
        output='screen',
        parameters=[params_file] # 나중에 파라미터 파일 로드
    )

    # (향후) 자율주행 노드들 실행
    # b. robot_localization (EKF 센서 퓨전)
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_params_file]
    )

    # parking_node = Node(...)
    # path_return_node = Node(...)

    # 4. 모든 것을 하나의 LaunchDescription에 담아 반환
    return LaunchDescription([
        teleop_launch,
        # socketcan_launch,
	socketcan_sender_launch,
        socketcan_receiver_launch,
        core_controller_node,
	robot_localization_node,
        # parking_node,
        # path_return_node,
    ])
