# 1. ROS 2 Humble 기본 이미지를 베이스로 사용
FROM ros:humble

# 2. 작업 디렉토리를 /root/ros_ws로 설정 (이후 모든 명령어의 기준 경로가 됨)
WORKDIR /root/ros_ws

# 3. 필요한 시스템 패키지 및 ROS 패키지 일괄 설치
#    - git, python3-pip: 기본 개발 도구
#    - can-utils: CAN 버스 테스트용 도구
#    - ros-humble-joy: 조이스틱 노드
#    - ros-humble-can-msgs, ros-humble-socketcan-interface: ros2_socketcan 관련
#    - libyaml-cpp-dev: C++에서 YAML 파일을 파싱하기 위한 라이브러리
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-pip \
    can-utils \
    libyaml-cpp-dev \
    nano \
    i2c-tools \
    libi2c-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-joy \
    ros-humble-can-msgs \
    ros-humble-ros2-socketcan \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    && rm -rf /var/lib/apt/lists/*

# 4. (선택적) 필요한 Python 라이브러리 설치
# RUN pip install pyserial 
RUN pip install --no-cache-dir pyyaml

# 5. 소스 코드를 컨테이너 내부로 복사
#    이제 src 폴더 전체를 복사하여 모든 패키지를 한 번에 추가합니다.
COPY src /root/ros_ws/src

# 6. ROS 의존성 설치 및 전체 소스 코드 빌드
#    (WORKDIR가 /root/ros_ws이므로 'cd ros_ws'가 필요 없어짐)
RUN . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src -y --ignore-src \
    && colcon build --symlink-install

# 7. 컨테이너 시작 시 작업 공간 자동 source
#    (이전 sed 명령어보다 더 안전한 방식으로 파일 끝에 추가)
RUN echo "source /root/ros_ws/install/setup.bash" >> /root/.bashrc
