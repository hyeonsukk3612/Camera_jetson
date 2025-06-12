***

예제 1 번의 섭스크라이버 노드에서 구독한 영상을 동영상 파일 (mp 4
로 저장하는 패키지 camera 1-2 를 작성하라
실행시 저장을 시작하고 ctrl+c 를 누르면 저장을 종료하도록 하라
예제 1 번과 같이 ros 명령어로 실행 결과를 확인하라

***

설정 및 빌드

***

colcon build --symlink-install --packages-select camera1-2

source install/setup.bash

***

젝슨보드

***

ros2 run camera1-2 pub

ros2 run camera1-2 sub_jetson

***

윈도우

***

ros2 run camera1-2 sub_wsl

gst-launch-1.0 -v udpsrc port=9005 ! ‘application/x-rtp,encodingname=(string)H264,payload=(int)96’ ! rtph264depay ! queue ! avdec_h264 ! videoconvert ! autovideosink

***

카메라 실제 구동 영상입니다

***

[https://youtube.com/shorts/9Q0vdCniHcU](https://youtu.be/ZGGesoSqcWg)
