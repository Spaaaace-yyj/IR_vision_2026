#ifndef CAR_CONTROL_NODE_HPP_
#define CAR_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <chrono>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

class CarControlNode : public rclcpp::Node
{
public:
        CarControlNode();

        void imageCallback();

        void debugThread();
private:

        cv::Ptr<cv::aruco::Dictionary> detector_dict_;
        cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
private:
        cv::VideoCapture cap_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
        rclcpp::TimerBase::SharedPtr image_timer_;

        //debug线程
        std::mutex img_mutex_;
        cv::Mat latest_frame_;

        std::thread show_thread_;
};


#endif // CAR_CONTROL_NODE_HPP