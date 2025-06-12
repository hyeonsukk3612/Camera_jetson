#include "rclcpp/rclcpp.hpp"                        // ROS 2 핵심 라이브러리 포함
#include "sensor_msgs/msg/compressed_image.hpp"     // 압축 이미지 메시지 타입
#include "opencv2/opencv.hpp"                       // OpenCV 이미지 처리 라이브러리
#include <memory>                                   // 스마트 포인터(std::shared_ptr 등) 사용
#include <functional>                               // std::bind, std::function 사용
#include <iostream>                                 // 표준 입출력

using std::placeholders::_1;                        // std::bind의 자리 표시자 사용

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{                                                   // 이미지 수신 시 실행되는 콜백 함수
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR); // 압축 이미지를 OpenCV Mat으로 변환
    cv::Mat gray_frame;                             // 그레이스케일 변환용 변수
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY); // 컬러 이미지를 그레이스케일로 변환
    cv::Mat binary_frame;                           // 이진화 결과 저장 변수
    cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY); // 그레이스케일 이미지 이진화
    cv::imshow("Original Image", frame);            // 원본 이미지 창 띄우기
    cv::imshow("Grayscale Image", gray_frame);      // 그레이스케일 이미지 창 띄우기
    cv::imshow("Binary Image", binary_frame);       // 이진화 이미지 창 띄우기
    cv::waitKey(1);                                 // 창 갱신 및 이벤트 처리(1ms 대기)
    RCLCPP_INFO(node->get_logger(), "Received Image: Format: %s, Size: (%d, %d)", 
                msg->format.c_str(), frame.rows, frame.cols); // 이미지 정보 로그 출력
}

int main(int argc, char* argv[])
{                                                   // 프로그램 진입점
    rclcpp::init(argc, argv);                       // ROS 2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl"); // 노드 생성(이름: camsub_wsl)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); // QoS 설정(최근 10개, Best Effort)
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn; // 콜백 함수 타입 선언
    fn = std::bind(mysub_callback, node, _1);       // 콜백 함수에 노드 바인딩
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);       // 이미지 토픽 구독자 생성
    rclcpp::spin(node);                             // 노드 실행(메시지 수신 및 콜백 처리)
    rclcpp::shutdown();                             // ROS 2 종료
    return 0;                                       // 프로그램 종료
}
