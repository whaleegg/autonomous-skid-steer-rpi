from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # teleop.launch.py 포함 (joy_node, teleop_node 실행)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bringup'), 'launch', 'teleop.launch.py')
        )
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

    # imu.launch.py 포함 (imu_driver, imu_filter 실행)
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bringup'), 'launch', 'imu.launch.py')
        )
    )

    return LaunchDescription([
        teleop_launch,
        socketcan_sender_launch,
	socketcan_receiver_launch,
        #imu_launch,
    ])
