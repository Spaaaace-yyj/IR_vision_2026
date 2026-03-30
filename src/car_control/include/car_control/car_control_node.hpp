#ifndef CAR_CONTROL_NODE_HPP_
#define CAR_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
// #include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <chrono>
#include <mutex>
#include <thread>

struct TagPose {
    int id;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Point3f center;
};

struct CubeState {
    int id;
    std::vector<TagPose> tags;

    cv::Point3f center;
    cv::Mat R;

    bool valid = false;
};

using namespace std::chrono_literals;

class CarControlNode : public rclcpp::Node
{
public:
        CarControlNode();

        void imageCallback();

        std::unordered_map<int, std::vector<int>> groupById(
            const std::vector<int>& ids);

        CubeState estimateCube(
            int id,
            const std::vector<int>& indices,
            const std::vector<std::vector<cv::Point2f>>& corners,
            const std::vector<cv::Point3f>& tag_def_point,
            const cv::Mat& camera_matrix,
            const cv::Mat& dist_coeffs);

        void drawCubes(
            cv::Mat& frame,
            const std::vector<CubeState>& cubes,
            const cv::Mat& camera_matrix,
            const cv::Mat& dist_coeffs);

        void debugThread();
private:
        cv::Mat camera_matrix_;   
        cv::Mat dist_coeffs_;    

        cv::Ptr<cv::aruco::Dictionary> detector_dict_;
        cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

        float tag_size = 0.08; //m
        float cube_size = 0.15; //m
        std::vector<cv::Point3f> tag_def_point;
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