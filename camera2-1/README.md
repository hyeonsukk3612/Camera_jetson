***

과제1
예제 1 번을 수정하여 WSL 2 의 subscriber 노드에서 영상원본을 그레
이영상 이진영상으로 각각 변환하고 3 가지 영상을 모두 출력하는
패키지 camera 2-1 을 작성하라
퍼블리셔 노드는 Jetson 보드에서 camera 패키지의 pub 노드를 실
행하라
예제 1 번과 같이 ros 2 명령어로 토픽전송상태를 확인하라

***

설정 및 빌드

***

colcon build --symlink-install --packages-select camera2-1

source install/setup.bash

***

젝슨보드

***

ros2 run camera2-1 pub

ros2 run camera2-1 sub

ros2 run camera2-1 sub_jetson

***

윈도우

***

gst-launch-1.0 -v udpsrc port=9005 ! ‘application/x-rtp,encodingname=(string)H264,payload=(int)96’ ! rtph264depay ! queue ! avdec_h264 ! videoconvert ! autovideosink

***

카메라 실제 구동 영상입니다

***

[https://youtube.com/shorts/ymdy2WEYDVQ](https://youtube.com/shorts/ymdy2WEYDVQ)


