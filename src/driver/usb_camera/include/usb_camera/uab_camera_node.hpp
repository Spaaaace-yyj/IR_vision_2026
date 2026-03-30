#ifndef USB_CAMERA_NODE_HPP_
#define USB_CAMERA_NODE_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <chrono>

using namespace std::chrono_literals;

class USBCameraNode : public rclcpp::Node
{
public:
  USBCameraNode();

  void USBCameraCallback();

private:

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image01_publisher_;
  rclcpp::TimerBase::SharedPtr image_timer_;

};

#endif // USB_CAMERA_NODE_HPP_