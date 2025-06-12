cmake_minimum_required(VERSION 3.16)
project(camera1-1)  # 폴더명과 일치하게 수정

# C++14 표준 사용
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# pub 노드 빌드
add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp sensor_msgs cv_bridge OpenCV)

# sub_jetson 노드 빌드
add_executable(sub_jetson src/sub_jetson.cpp)
ament_target_dependencies(sub_jetson rclcpp sensor_msgs cv_bridge OpenCV) # cv_bridge 추가

# sub_wsl 노드 빌드
add_executable(sub_wsl src/sub_wsl.cpp)
ament_target_dependencies(sub_wsl rclcpp sensor_msgs cv_bridge OpenCV) # cv_bridge 추가

# 실행 파일 설치
install(TARGETS
  pub
  sub_jetson
  sub_wsl
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
