# bringup/launch/autonomy.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # pure_pursuit 노드가 사용할 파라미터 파일 경로
    # TODO: params_package/config에 pure_pursuit.yaml 파일 생성 필요
    pp_params_file = os.path.join(
        get_package_share_directory('params_package'),
        'config',
        'pure_pursuit.yaml'
    )

    # 1. 경로 기록 노드 (우리 자작 노드)
    path_recorder_node = Node(
        package='autonomy_package',
        executable='path_recorder_node',
        name='path_recorder_node',
        output='screen'
    )

    # 2. 경로 스무딩 노드 (오픈소스)
    path_smoother_node = Node(
        package='path_smoother', # <-- 간결해진 이름 사용
        executable='cubic_spline_node', # 실제 실행 파일 이름 확인 필요
        name='path_smoother_node',
        output='screen',
#	parameters=[pp_params_file],
#        remappings=[
#            ('input_path', '/recorded_path'),
#            ('tgt_path', '/smoothed_path')
#        ]
    )

    path_publisher_node = Node(
        package='path_smoother',          # 1. package.xml의 <name> 태그와 일치하는가?
        executable='path_publisher', # 2. CMakeLists.txt의 add_executable() 이름과 일치하는가?
        name='path_publisher_node',
        output='screen'
    )

    # 3. 경로 추종 노드 (오픈소스)
    pure_pursuit_node = Node(
        package='pure_pursuit_planner', # <-- 간결해진 이름 사용
        executable='pure_pursuit_planner', # 실제 실행 파일 이름 확인 필요
        name='pure_pursuit_node',
        output='screen',
	parameters=[pp_params_file],
        remappings=[
            ('odom', '/odom/wheel'),
	    ('tgt_path', '/tgt_path'),
            ('cmd_vel', '/cmd_vel_auto')    # 출력 토픽 이름 확인 필요
        ]
    )

    return LaunchDescription([
        path_recorder_node,
        path_smoother_node,
	path_publisher_node,
        pure_pursuit_node,
    ])
