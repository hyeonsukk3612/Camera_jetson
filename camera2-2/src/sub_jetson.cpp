#include "rclcpp/rclcpp.hpp"                           // ROS 2 핵심 라이브러리
#include "sensor_msgs/msg/compressed_image.hpp"        // ROS 압축 이미지 메시지
#include "opencv2/opencv.hpp"                          // OpenCV 이미지 처리 라이브러리
#include <memory>                                      // 스마트 포인터 사용
#include <functional>                                  // std::bind 등 함수 객체 사용
#include <iostream>                                    // 표준 입출력
#include <vector>                                      // std::vector 사용

using std::placeholders::_1;                           // std::bind 자리 표시자

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=192.168.0.13 port=8001 sync=false";   // GStreamer 파이프라인 정의

cv::VideoWriter writer;                                // 영상 전송용 VideoWriter 객체

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{                                                      // 이미지 수신 콜백 함수
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR); // 압축 이미지 디코딩
    if (frame.empty()) {                               // 디코딩 실패 시 에러 출력
        RCLCPP_ERROR(node->get_logger(), "Failed to decode image");
        return;
    }
    cv::Mat gray;                                      // 그레이스케일 변환용 변수
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);     // 컬러 이미지를 그레이스케일로 변환
    cv::Mat binary;                                    // 이진화 결과 저장 변수
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU); // OTSU 알고리즘으로 이진화
    writer << binary;                                  // 이진화 영상을 GStreamer로 전송
    RCLCPP_INFO(node->get_logger(), "Received Image: %s, %d x %d, Binary Image Size: %d x %d", 
                msg->format.c_str(), frame.cols, frame.rows, binary.cols, binary.rows); // 로그 출력
}

int main(int argc, char * argv[])
{                                                      // 프로그램 진입점
    rclcpp::init(argc, argv);                          // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("camera1_1_node"); // 노드 생성
    writer.open(dst, 0, (double)30, cv::Size(640, 360), false); // GStreamer 파이프라인 연결, 흑백 영상(false)
    if (!writer.isOpened()) {                          // 파이프라인 오픈 실패 시 에러 출력 후 종료
        RCLCPP_ERROR(node->get_logger(), "Writer open failed!");
        rclcpp::shutdown();
        return -1;
    }
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); // QoS 설정(최근 10개, best_effort)
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn; // 콜백 함수 타입 선언
    fn = std::bind(mysub_callback, node, _1);         // 콜백 함수에 노드 바인딩
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);         // 이미지 토픽 구독자 생성
    rclcpp::spin(node);                               // 노드 실행(메시지 수신 및 콜백 처리)
    rclcpp::shutdown();                               // ROS 2 종료
    return 0;                                         // 프로그램 종료
}

