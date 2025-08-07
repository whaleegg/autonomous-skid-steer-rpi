import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 파라미터 파일들의 경로를 정의
    
    # ros2_odometry_estimation 노드가 사용할 파라미터 파일
    # (wheel_radius, track_width 등이 포함된 파일)
    vehicle_params_file = os.path.join(
        get_package_share_directory('params_package'),
        'config',
        'vehicle_params.yaml'
    )

    # robot_localization 노드가 사용할 파라미터 파일
    ekf_params_file = os.path.join(
        get_package_share_directory('params_package'),
        'config',
        'ekf.yaml'
    )
    
    # 2. 오도메트리 파이프라인의 각 노드를 선언

    # 노드 1: 엔코더 RPM -> /odom/wheel 변환
    odometry_estimation_node = Node(
        package='odometry_estimator',
        executable='odometry_estimator',
        name='odometry_estimation_node',
        output='screen',
        parameters=[vehicle_params_file],
        remappings=[
            ('odom', '/odom/wheel')
        ]
    )

    # 노드 2: /odom/wheel + /imu/data -> /odometry/filtered 융합
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[ekf_params_file]
    )
    
    # 3. 두 노드를 모두 실행하도록 LaunchDescription 반환
    return LaunchDescription([
        odometry_estimation_node,
        robot_localization_node,
    ])
