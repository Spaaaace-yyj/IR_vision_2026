#ifndef CAR_CONTROL_NODE_HPP_
#define CAR_CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "auto_aim_interfaces/msg/cmd.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>

#include "tag_detector.hpp"
#include "auto_aim_interfaces/msg/mcu_feed_back.hpp"

using namespace std::chrono_literals;

//多线程通讯
struct DebugPacket {
    rclcpp::Time stamp;

    cv::Mat frame;
    std::vector<cv::Point3f> laser_points;
    CubeDetectionResult result;
    double detector_latency;
};

class CarControlNode : public rclcpp::Node
{
public:
    CarControlNode();
    ~CarControlNode() override;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    void mcuCallback(const auto_aim_interfaces::msg::McuFeedBack::SharedPtr msg);

    void drawCubes(
    cv::Mat& frame,
    const std::vector<CubeState>& cubes,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs);

    static cv::Point2f polarToCartesian(float range, float angle_rad);

    static std::vector<cv::Point2f> laserScanToCartesian(const sensor_msgs::msg::LaserScan& scan);

    static cv::Mat buildLaserScanView(
        const std::vector<cv::Point3f>& points,
        const std::vector<CubeState>& cubes,
        int image_size = 1000);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void debugThread();

private:
    void bindThreadToCores(const std::vector<int>& cores);

    static cv::Matx33d rpyToRotationMatrix(const cv::Vec3d& rpy);

    static cv::Point3f transformPoint(
    const cv::Point3f& point,
    const cv::Matx33d& rotation,
    const cv::Vec3d& translation);

    std::vector<cv::Point3f> transformLaserPointsToBase(
    const std::vector<cv::Point2f>& laser_points) const;

    float projectAndComputeAngle(const cv::Point3f& p);

    CubeDetectParams cube_detector_params;
    std::unique_ptr<Detector> cube_detector_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv::Matx33d camera_to_base_rotation_ = cv::Matx33d::eye();
    cv::Vec3d camera_to_base_translation_ = cv::Vec3d(0.0, 0.0, 0.0);
    cv::Matx33d lidar_to_base_rotation_ = cv::Matx33d::eye();
    cv::Vec3d lidar_to_base_translation_ = cv::Vec3d(0.0, 0.0, 0.0);

    float tag_size = 0.08; //m
    float cube_size = 0.15; //m

    CubeDetectionResult cube_state_;
    bool is_blue = false;

private:
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr scan_image_publisher_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Cmd>::SharedPtr control_cmd_publisher_;

    rclcpp::Subscription<auto_aim_interfaces::msg::McuFeedBack>::SharedPtr mcu_feedback_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    std::shared_ptr<image_transport::Subscriber> img_sub_;

    //debug线程
    std::thread show_thread_;
    std::mutex debug_mutex_;
    std::atomic<bool> debug_running_{true};

    DebugPacket debug_buffer[2];
    std::atomic<int> debug_write_index_{0};
    std::atomic<int> debug_read_index_{1};
    std::atomic<uint64_t> debug_frame_id_{0};
    std::atomic<uint64_t> last_debug_processed_id_{0};
    std::atomic<bool> new_frame_{false};

    std::vector<CubeState> cubes_;
    std::vector<CubeState> cubes_base_;
    std::vector<cv::Point3f> laser_points_base_;

    bool debug_mode_ = true;
};


#endif // CAR_CONTROL_NODE_HPP
