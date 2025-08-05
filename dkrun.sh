#!/bin/bash

# --- 설정 변수 ---
# 우리의 소스 코드가 위치한 절대 경로를 여기에 명시합니다.
# ~ 기호는 스크립트에서 항상 동작하지 않을 수 있으므로, $HOME 환경 변수를 사용합니다.
SCRIPT_DIR="$HOME/dkbuild/js"

# --- X11 포워딩 설정 (호스트 RPi에서 실행) ---
xhost +local:docker

# --- Docker 실행 명령어 정의 ---
CMD="docker run \
    --rm \
    -it \
    --privileged \
    --net host \
    --name skid_steer_dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \

    -e XAUTHORITY=/tmp/.Xauthority \
    -v $HOME/.Xauthority:/tmp/.Xauthority:rw \

    -v ${SCRIPT_DIR}/src:/root/ros_ws/src \
    ssv-ros:dev"

# --- 명령어 실행 ---
echo "========================================================"
echo "Starting Skid-Steer ROS Development Container (with GUI support)..."
echo "Image: ssv-ros:dev"
echo "Host source directory: ${SCRIPT_DIR}/src"
echo "Container workspace: /root/ros_ws/src"
echo "Executing command:"
echo $CMD
echo "========================================================"

# 실제로 Docker 컨테이너를 실행
$CMD

# --- 컨테이너 종료 후 정리 ---
echo "Container stopped. Revoking X server access for docker."
xhost -local:docker
