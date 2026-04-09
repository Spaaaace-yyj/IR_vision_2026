#ifndef CAR_CONTROL_NODE_HPP_
#define CAR_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
// #include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <chrono>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

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

    cv::Mat rvec;
    cv::Mat tvec;

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

        static cv::Point2f polarToCartesian(float range, float angle_rad);

        static std::vector<cv::Point2f> laserScanToCartesian(
            const sensor_msgs::msg::LaserScan& scan);

        static cv::Mat buildLaserScanView(
            const std::vector<cv::Point3f>& points,
            const std::vector<CubeState>& cubes,
            int image_size = 1000);

        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void debugThread();
private:
        static cv::Matx33d rpyToRotationMatrix(const cv::Vec3d& rpy);

        static cv::Point3f transformPoint(
            const cv::Point3f& point,
            const cv::Matx33d& rotation,
            const cv::Vec3d& translation);

        CubeState transformCubeToBase(const CubeState& cube) const;

        std::vector<cv::Point3f> transformLaserPointsToBase(
            const std::vector<cv::Point2f>& laser_points) const;

        cv::Mat camera_matrix_;   
        cv::Mat dist_coeffs_;    

        cv::Matx33d camera_to_base_rotation_ = cv::Matx33d::eye();
        cv::Vec3d camera_to_base_translation_ = cv::Vec3d(0.0, 0.0, 0.0);
        cv::Matx33d lidar_to_base_rotation_ = cv::Matx33d::eye();
        cv::Vec3d lidar_to_base_translation_ = cv::Vec3d(0.0, 0.0, 0.0);

        cv::Ptr<cv::aruco::Dictionary> detector_dict_;
        cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
        cv::Ptr<cv::aruco::DetectorParameters> fast_detector_params_;

        float tag_size = 0.08; //m
        float cube_size = 0.15; //m
        float roi_search_scale_ = 0.75F;
        float roi_padding_ratio_ = 0.08F;
        int roi_min_area_ = 500;
        int roi_child_min_area_ = 600;
        
        std::vector<cv::Point3f> tag_def_point;
private:
        cv::VideoCapture cap_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
        rclcpp::TimerBase::SharedPtr image_timer_;

        //debug线程
        std::mutex img_mutex_;
        cv::Mat latest_frame_;

        std::vector<CubeState> cubes_;
        std::vector<CubeState> cubes_base_;
        std::vector<cv::Point3f> laser_points_base_;

        std::thread show_thread_;

        std::thread control_thread_;

};


#endif // CAR_CONTROL_NODE_HPP
