***

예제 1 번의 subscriber 노드를 수정하여 수신한 영상을 컬러영상 그
레이영상 이진영상으로 변환하고 이진영상을 PC 로 전송하여 출력하
는 패키지 camera 1-1 을 작성하라
writer open dst 0 double 30 cv Size 640 360 false
퍼블리셔 노드는 Jetson 보드에서 camera 패키지의 pub 노드를 실행하
라
예제 1 번과 같이 ros 명령어로 실행 결과를 확인하라

***

설정 및 빌드

***

colcon build --symlink-install --packages-select camera1-1

source install/setup.bash

***

젝슨보드

***

ros2 run camera1-1 pub

ros2 run camera1-1 sub_jetson

***

윈도우

***

ros2 run camera1-1 sub_wsl

gst-launch-1.0 -v udpsrc port=9005 ! ‘application/x-rtp,encodingname=(string)H264,payload=(int)96’ ! rtph264depay ! queue ! avdec_h264 ! videoconvert ! autovideosink

***

카메라 실제 구동 영상입니다

***

[https://youtube.com/shorts/ymdy2WEYDVQ?feature=share](https://youtube.com/shorts/9Q0vdCniHcU)
