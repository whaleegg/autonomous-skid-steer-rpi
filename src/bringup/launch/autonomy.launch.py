# bringup/launch/autonomy.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. 경로 기록 노드 (우리 자작 노드)
    path_recorder_node = Node(
        package='autonomy_package',
        executable='path_recorder_node',
        name='path_recorder_node',
        output='screen'
    )
    
#    # 2. 경로 스무딩 노드 (오픈소스)
#    path_smoother_node = Node(
#        package='path_smoother', # <-- 간결해진 이름 사용
#        executable='cubic_spline_node', # 실제 실행 파일 이름 확인 필요
#        name='path_smoother_node',
#        output='screen',
#        remappings=[
#            ('input_path', '/recorded_path'),
#            ('tgt_path', '/smoothed_path')
#        ]
#    )

    # 2. 경로 뒤집기 및 재발행 노드
    path_return_node = Node(
        package='autonomy_package',
        executable='path_return_node',
        name='path_return_node',
        output='screen',
        remappings=[
            # 이 노드의 출력을 pure_pursuit의 입력 토픽 이름인 'local_traj'로 변경
            ('tgt_traj', '/local_traj')
        ]
    )

    # 3. 경로 추종 노드 (오픈소스)
    pure_pursuit_node = Node(
        package='pure_pursuit_planner', # <-- 간결해진 이름 사용
        executable='pure_pursuit_planner', # 실제 실행 파일 이름 확인 필요
        name='pure_pursuit_node',
        output='screen',
        remappings=[
            ('odom', '/odom/wheel'),
            ('local_traj', '/local_traj'), # 입력 토픽 이름 확인 필요
            ('ctrl_cmd', '/cmd_vel_auto')    # 출력 토픽 이름 확인 필요
        ]
    )

    return LaunchDescription([
        path_recorder_node,
        #path_smoother_node,
	path_return_node,
        pure_pursuit_node,
    ])
