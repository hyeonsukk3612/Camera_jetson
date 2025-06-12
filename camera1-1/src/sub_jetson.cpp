#include "rclcpp/rclcpp.hpp"                   // ROS2 C++ 라이브러리
#include "sensor_msgs/msg/compressed_image.hpp" // ROS2 압축 이미지 메시지
#include "opencv2/opencv.hpp"                   // OpenCV 이미지 처리 라이브러리
#include <memory>                               // 스마트 포인터 사용
#include <functional>                           // std::bind, std::function 사용
#include <iostream>                             // 표준 입출력
#include <signal.h>                             // 시그널 처리

using std::placeholders::_1;                    // std::bind에서 사용

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! "
                  "nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! "
                  "h264parse ! rtph264pay pt=96 ! "
                  "udpsink host=203.234.58.168 port=9005 sync=false"; // GStreamer 파이프라인

cv::VideoWriter writer;                         // 영상 저장/전송을 위한 VideoWriter 객체

bool stop_video = false;                        // 영상 전송 중지 플래그

// 시그널 핸들러: Ctrl+C 등으로 프로그램 종료 시 영상 전송 중지
void signal_handler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received. Stopping video recording..." << std::endl;
    stop_video = true;
    writer.release(); // VideoWriter 종료
}

// 이미지 메시지 콜백 함수
void mysub_callback(rclcpp::Node::SharedPtr node, 
                   const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    if (stop_video) {
        return; // 영상 전송 중지 시 콜백 무시
    }

    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR); // 압축 이미지 디코딩

    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY); // 컬러 이미지를 그레이스케일로 변환

    cv::Mat gray_bgr;
    cv::cvtColor(gray_frame, gray_bgr, cv::COLOR_GRAY2BGR); // 그레이스케일을 다시 BGR로 변환(OpenCV VideoWriter 호환)

    writer << gray_bgr; // 영상으로 저장/전송

    RCLCPP_INFO(node->get_logger(), 
               "Received Image (Grayscale): %s, %d, %d", 
               msg->format.c_str(), 
               gray_bgr.rows, 
               gray_bgr.cols); // 로그 출력
}

int main(int argc, char* argv[])
{
    signal(SIGINT, signal_handler); // Ctrl+C 시그널 등록

    rclcpp::init(argc, argv); // ROS2 초기화
    auto node = std::make_shared<rclcpp::Node>("camsub"); // 노드 생성

    // VideoWriter 초기화 (GStreamer 파이프라인 사용, 640x360, 30fps, BGR 컬러)
    writer.open(dst, 0, (double)30, cv::Size(640, 360), true);
    if(!writer.isOpened()) { 
        RCLCPP_ERROR(node->get_logger(), "Writer open failed!"); // 초기화 실패 시 에러 로그
        rclcpp::shutdown(); 
        return -1; 
    }
    
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); // QoS 설정
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1); // 콜백 함수 바인딩
    
    // 이미지 토픽 구독
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed",
        qos_profile,
        fn
    );
    
    rclcpp::spin(node); // 노드 실행
    rclcpp::shutdown(); // 종료
    return 0;
}
