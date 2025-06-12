#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <signal.h>

using std::placeholders::_1;

std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! "
                  "nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! "
                  "h264parse ! rtph264pay pt=96 ! "
                  "udpsink host=203.234.58.168 port=9005 sync=false";

cv::VideoWriter writer;

// Flag to check if signal is received to stop saving video
bool stop_video = false;

// Signal handler for SIGINT (Ctrl+C)
void signal_handler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received. Stopping video recording..." << std::endl;
    stop_video = true;
    writer.release(); // Close the video writer
}

// Callback for image subscription
void mysub_callback(rclcpp::Node::SharedPtr node, 
                   const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    if (stop_video) {
        return; // Do nothing if the signal to stop video saving is received
    }

    // Decode the compressed image to an OpenCV Mat
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    // Convert the image to grayscale
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

    // Convert grayscale image back to BGR for compatibility with the video writer
    cv::Mat gray_bgr;
    cv::cvtColor(gray_frame, gray_bgr, cv::COLOR_GRAY2BGR);

    // Write the grayscale image to the video
    writer << gray_bgr;

    // Log the image details
    RCLCPP_INFO(node->get_logger(), 
               "Received Image (Grayscale): %s, %d x %d", 
               msg->format.c_str(), 
               gray_bgr.rows, 
               gray_bgr.cols);
}

int main(int argc, char* argv[])
{
    // Set up signal handling for Ctrl+C (SIGINT)
    signal(SIGINT, signal_handler);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("camsub");

    // Open the VideoWriter to save the video in MP4 format
    writer.open("output_video.mp4", cv::VideoWriter::fourcc('H', '2', '6', '4'), 30.0, cv::Size(640, 360), true);
    if (!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Writer open failed!");
        rclcpp::shutdown();
        return -1;
    }

    // Create QoS profile and the subscription callback
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    // Create the subscription to receive compressed images
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed",
        qos_profile,
        fn
    );

    // Spin the node to keep receiving and processing messages
    rclcpp::spin(node);

    // Shutdown ROS and clean up
    rclcpp::shutdown();
    return 0;
}
