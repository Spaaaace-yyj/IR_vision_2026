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

#include <eigen3/Eigen/Dense>

#include "tag_detector.hpp"
#include "auto_aim_interfaces/msg/mcu_feed_back.hpp"

#define DEG2RED (M_PI / 180.0f)
#define RED2DEG (180.0f / M_PI)
using namespace std::chrono_literals;

enum RobotState {
    SEARCHING_TARGET,
    TRACING,    //单片机执行target_yaw对应的yaw,速度还是由单片机控制
    SEARCH_TABLE,   //掉台后，单片机把角度和速度控制权交给小电脑
    READY_TO_PUSH,  //这个看是还是由小电脑控制还是由
};

enum SearchTableState {
    IDLE,
    ALIGN_WALL,     //对齐当前的墙，目标值直接为墙角度(垂直墙)
    CHECK_IS_INSIDE,
    CHANGE_WALL,    //切换墙，目标值为设定的固定角度
    TURN_ROUND,
    MOVE_TO_CENTER,
    ADJUST_DISTANCE,
};

enum MCUFeedState {
    ON_TABLE,
    FALL_TABLE,
};

enum AxisDirection
{
    POS_X,
    NEG_X,
    POS_Y,
    NEG_Y
};

struct CarControlCmd {
    float target_yaw;
    float target_speed;
    float wall_yaw_offset;
};

struct LineFeature
{
    bool valid = false;

    Eigen::Vector2f center;

    Eigen::Vector2f tangent;
    Eigen::Vector2f normal;

    float length = 0.0f;

    int cluster_id = -1;
};

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

    //callback
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    void mcuCallback(const auto_aim_interfaces::msg::McuFeedBack::SharedPtr msg);

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void updateFSM();

    void updateEvent();

    void changeState(RobotState new_state);

    void updateTableFSM();

    void updateUpTableEvent();

    void changeUpTableState(SearchTableState new_state);

    void pubTargetCube();

    static cv::Point2f polarToCartesian(float range, float angle_rad);

    std::vector<cv::Point2f> laserScanToCartesian(const sensor_msgs::msg::LaserScan& scan);

    static cv::Matx33d rpyToRotationMatrix(const cv::Vec3d& rpy);

    static cv::Point3f transformPoint(
    const cv::Point3f& point,
    const cv::Matx33d& rotation,
    const cv::Vec3d& translation);

    std::vector<cv::Point3f> transformLaserPointsToBase(
    const std::vector<cv::Point2f>& laser_points) const;

    LineFeature fitLinePCA(const std::vector<cv::Point2f>& cluster);

    float projectAndComputeAngle(
        const cv::Point3f &p,
        AxisDirection axis);


    float projectAndComputeAngle(
        const Eigen::Vector2f &p,
        AxisDirection axis);

    std::vector<std::vector<cv::Point2f> > clusterLaserPoints(
    const std::vector<cv::Point2f>& points,
    float threshold);

    cv::Point2f transformPointToYawWorld(
        const cv::Point2f &p,
        float yaw);

    std::vector<cv::Point2f>transformPointsToYawWorld(
        const std::vector<cv::Point2f> &points,
        float yaw);

    float pointLineDistance(
    const cv::Point2f& p,
    const cv::Point2f& a,
    const cv::Point2f& b);

    bool CheckFindTarget() const;

    //debug function
    void debugThread();

    void drawX(cv::Point2f p, cv::Mat& frame);

    static cv::Mat buildLaserScanView(
    const std::vector<cv::Point3f>& points,
    const std::vector<std::vector<cv::Point2f>>& clusters,
    const std::vector<LineFeature>& lines,
    const std::vector<CubeState>& cubes,
    int image_size = 1000);

    void drawCubes(
        cv::Mat &frame,
        const std::vector<CubeState> &cubes,
        const cv::Mat &camera_matrix,
        const cv::Mat &dist_coeffs);

    std::string stateToString(RobotState state);

    void DebugFSMStateChangeInfo(RobotState new_state);

    void bindThreadToCores(const std::vector<int>& cores);

private:

    RobotState current_state = SEARCH_TABLE;
    MCUFeedState feed_state = ON_TABLE;
    SearchTableState search_table_state = IDLE;

    CarControlCmd car_cmd = {
        .target_yaw = 0.0f,
        .target_speed = 0.0f,
        .wall_yaw_offset = 0.0f,
    };

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
    float target_yaw_ = 0.0f;
    float is_tracing_ = -1.0f;
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
    std::vector<std::vector<cv::Point2f>> cluster_points_;
    std::vector<LineFeature> point_line_;

    float car_yaw_rad = 0.0f;
    float car_yaw_deg = 0.0f;

    bool down_state = false;

    bool debug_mode_ = true;
};


#endif // CAR_CONTROL_NODE_HPP
