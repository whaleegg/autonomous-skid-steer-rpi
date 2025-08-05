cmake_minimum_required(VERSION 3.8)
project(controller_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- 1. 필요한 ROS 2 패키지 및 라이브러리 찾기 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED) # CAN 메시지 사용을 위해 필요
find_package(interfaces REQUIRED) # 우리가 만든 커스텀 메시지 사용을 위해 필요

# --- 2. 실행 파일(노드) 빌드 설정 ---
# 'core_controller_node'라는 이름의 실행 파일을 'src/core_controller_node.cpp'로부터 생성
add_executable(core_controller_node src/core_controller_node.cpp)

# --- 3. 실행 파일에 필요한 라이브러리 링크 ---
# core_controller_node가 rclcpp, std_msgs, geometry_msgs, can_msgs, interfaces 패키지의 기능들을 사용할 수 있도록 링크
ament_target_dependencies(core_controller_node
  rclcpp
  std_msgs
  geometry_msgs
  can_msgs
  interfaces
)

# --- 4. 실행 파일 설치 ---
# 빌드된 실행 파일을 'lib/<패키지이름>/' 경로에 설치
install(TARGETS
  core_controller_node
  DESTINATION lib/${PROJECT_NAME})

# --- 5. Launch 파일 등 기타 리소스 설치 ---
# launch 폴더 안의 모든 파일을 'share/<패키지이름>/launch/' 경로에 설치
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

# --- 6. 테스트 설정 (선택 사항) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
    
# --- 7. 패키지 정보 등록 ---
# ROS 2 시스템이 이 패키지를 인식하도록 하는 기본 설정
ament_package()
