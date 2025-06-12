***

예제 1 번의 섭스크라이버 노드에서 구독한 영상을 화면에 출력하고
동시에 동영상 파일 (mp 4 로 저장하는 패키지 camera 2-2 를 작성하라
실행시 저장을 시작하고 ctrl+c 를 누르면 저장을 종료하도록 하라
예제 1 번과 같이 ros 2 명령어로 토픽전송상태를 확인하라

***

설정 및 빌드

***

colcon build --symlink-install --packages-select camera2-2

source ~/ros2_ws/install/local_setup.bash

***

젝슨보드

***

ros2 run camera2-2 pub

ros2 run camera2-2 sub

ros2 run camera2-2 sub_jetson

***

윈도우

***

gst-launch-1.0 -v udpsrc port=9005 ! ‘application/x-rtp,encodingname=(string)H264,payload=(int)96’ ! rtph264depay ! queue ! avdec_h264 ! videoconvert ! autovideosink

***

카메라 실제 구동 영상입니다

***

[https://youtube.com/shorts/ymdy2WEYDVQ](https://youtube.com/shorts/ymdy2WEYDVQ)




