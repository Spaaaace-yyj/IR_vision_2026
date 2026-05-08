#include "../include/car_control/car_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

CarControlNode::CarControlNode() : Node("car_control_node")
{
    std::string transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
    img_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
    this, "/image_raw", std::bind(&CarControlNode::imageCallback, this, std::placeholders::_1),
    transport_, rmw_qos_profile_sensor_data));

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/car_control/image_debug", rclcpp::SensorDataQoS());
    
    scan_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/car_control/scan_debug", rclcpp::SensorDataQoS());
    control_cmd_publisher_ = this->create_publisher<auto_aim_interfaces::msg::Cmd>("control/car_cmd_vel", 1);

    mcu_feedback_sub_ = this->create_subscription<auto_aim_interfaces::msg::McuFeedBack>(
        "feedback/mcu_msg",
        10,
        std::bind(&CarControlNode::mcuCallback, this, std::placeholders::_1));
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&CarControlNode::scanCallback, this, std::placeholders::_1));
    
    tag_size = this->declare_parameter("tag_size", 0.08);
    cube_size = this->declare_parameter("cube_size", 0.15);

    const auto load_vec3_parameter = [this](const std::string& name) {
        std::vector<double> values = this->declare_parameter<std::vector<double>>(
            name, {0.0, 0.0, 0.0});

        if (values.size() != 3) {
            RCLCPP_WARN(
                this->get_logger(),
                "Parameter %s must contain exactly 3 elements, fallback to zeros.",
                name.c_str());
            values = {0.0, 0.0, 0.0};
        }

        return cv::Vec3d(values[0], values[1], values[2]);
    };

    camera_to_base_translation_ = load_vec3_parameter("camera_to_base.translation");
    camera_to_base_rotation_ =
        rpyToRotationMatrix(load_vec3_parameter("camera_to_base.rpy"));
    lidar_to_base_translation_ = load_vec3_parameter("lidar_to_base.translation");
    lidar_to_base_rotation_ =
        rpyToRotationMatrix(load_vec3_parameter("lidar_to_base.rpy"));

    // camera_matrix_ = (cv::Mat_<double>(3,3) << 
    //     504.504779, 0, 647.473284,
    //     0, 502.266322, 363.157317,
    //     0, 0, 1);

    // dist_coeffs_ = (cv::Mat_<double>(1,5) << 0.003130, 0.002283, 0.001391, 0.001827, 0.0);

    camera_matrix_ = (cv::Mat_<double>(3,3) << 
        1352.039377, 0, 642.538070,
        0, 1345.311392, 512.435347,
        0, 0, 1);

    dist_coeffs_ = (cv::Mat_<double>(1,5) << -0.072975, 0.200117, 0.002497, 0.003193, 0.0);

    cube_detector_params.camera_matrix_ = camera_matrix_;
    cube_detector_params.dist_coeffs_ = dist_coeffs_;
    cube_detector_params.camera_to_base_rotation_ = camera_to_base_rotation_;
    cube_detector_params.camera_to_base_translation_ = camera_to_base_translation_;
    cube_detector_params.cube_size_ = cube_size;
    cube_detector_params.tag_size_ = tag_size;

    cube_detector_ = std::make_unique<Detector>(cube_detector_params);

    if (debug_mode_) {
        show_thread_ = std::thread(&CarControlNode::debugThread, this);
    }

    RCLCPP_INFO(this->get_logger(), "Success Init Car Control node!");
}

CarControlNode::~CarControlNode()
{
    debug_running_.store(false);
    if (show_thread_.joinable()) {
        show_thread_.join();
    }
}

/**
 * @brief 图像回调函数
 * @param msg
 */
void CarControlNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    cv::Mat frame;
    try{
        // 转换为 cv::Mat
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        frame = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "cv_bridge exception: %s", e.what());
    }

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty frame");
        return;
    }

    if (!cube_detector_) {
        RCLCPP_ERROR(this->get_logger(), "Cube detector is not initialized.");
        return;
    }
    //detector耗时统计
    auto t0 = std::chrono::steady_clock::now();

    //apriltag识别和cube位姿解算
    const CubeDetectionResult detection = cube_detector_->detectCubes(frame);
    cube_state_ = detection;

    auto t1 = std::chrono::steady_clock::now();

    //图像处理系统总延迟统计
    double latency = (this->now() - msg->header.stamp).seconds() * 1000.0;
    double detect_time = std::chrono::duration<double, std::milli>(t1 - t0).count();

    static int fps = 0;
    static auto start_time = this->now();
    static double total_latency = 0;
    static double total_detect_time = 0;

    total_latency += latency;
    total_detect_time += detect_time;

    //延迟信息打印
    if(this->now() - start_time >= rclcpp::Duration::from_seconds(1.0)){
        RCLCPP_INFO(this->get_logger(), "FPS: %d | latency: %f | detect_latency: %f", fps, total_latency / fps, total_detect_time / fps);
        fps = 0;
        start_time = this->now();
        total_latency = 0;
        total_detect_time = 0;
    }
    fps++;

    //debug线程数据无锁拷贝
    if(debug_mode_)
    {
        int w = debug_write_index_.load(std::memory_order_release);

        DebugPacket pkt;
        pkt.frame = frame.empty() ? cv::Mat() : frame;
        pkt.result = detection;
        pkt.detector_latency = latency;
        pkt.stamp = this->now();

        {
            std::lock_guard<std::mutex> lock(debug_mutex_);
            pkt.laser_points = laser_points_base_;
        }
        debug_buffer[w] = pkt;

        debug_frame_id_.fetch_add(1, std::memory_order_release);

        int r = debug_read_index_.load(std::memory_order_relaxed);
        debug_write_index_.store(r, std::memory_order_relaxed);
        debug_read_index_.store(w, std::memory_order_relaxed);
    }
}

/**
 * @brief 单片机消息回调函数
 * @param msg
 */
void CarControlNode::mcuCallback(const auto_aim_interfaces::msg::McuFeedBack::SharedPtr msg) {

    CubeState target_cube;
    if (!cube_state_.cubes_base.empty()) {
        for (size_t i = 0; i < cube_state_.cubes_base.size(); i++) {
            if (cube_state_.cubes_base[i].id == 0) {
                target_cube = cube_state_.cubes_base[i];
                break;
            }
            if (cube_state_.cubes_base[i].id == 1 && is_blue) {
                target_cube = cube_state_.cubes_base[i];
            }
            if (cube_state_.cubes_base[i].id == 2 && !is_blue) {
                target_cube = cube_state_.cubes_base[i];
            }
        }
        target_yaw_ = projectAndComputeAngle(target_cube.center) * (180.0f / M_PI);
        is_tracing_ = target_cube.id;
    }else {
        target_yaw_ = 0.0f;
        is_tracing_ = -1.0f;
    }

    auto_aim_interfaces::msg::Cmd cmd_msg;
    cmd_msg.target_yaw = target_yaw_ + msg->yaw;
    while (cmd_msg.target_yaw > 180.0f) {
        cmd_msg.target_yaw -= 360.0f;
    }
    while (cmd_msg.target_yaw < -180.0f) {
        cmd_msg.target_yaw += 360.0f;
    }

    cmd_msg.tracing = is_tracing_;

    control_cmd_publisher_->publish(cmd_msg);
    car_yaw_rad = msg->yaw * (M_PI / 180.0f);
    // RCLCPP_INFO(this->get_logger(), "target_yaw = %f | is_tracing = %f", cmd_msg.target_yaw, is_tracing_);

}

/**
 *
 * @brief 雷达消息回调函数
 * @param msg
 */
void CarControlNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::vector<cv::Point2f> laser_points;
    if (down_state) {
        laser_points = transformPointsToYawWorld(laserScanToCartesian(*msg), car_yaw_rad);
    }else {
        laser_points = laserScanToCartesian(*msg);
    }

    //转换到底盘坐标系
    const std::vector<cv::Point3f> laser_points_base =
        transformLaserPointsToBase(laser_points);
    //点云聚类
    std::vector<std::vector<cv::Point2f>> cluster_points = clusterLaserPoints(laser_points, 0.10f);
    std::vector<LineFeature> point_line;

    for (size_t i = 0; i < cluster_points.size(); i++) {
        point_line.push_back(fitLinePCA(cluster_points[i]));
    }
    // int target_line_idx = 0;
    // float min_y = 10000.0f;
    // for (size_t i = 0; i < point_line.size(); i++) {
    //     if (point_line[i].center.x() < 0) {
    //         if (min_y > point_line[i].center.y()) {
    //
    //         }
    //     }
    // }
    if (debug_mode_)
    {
        std::lock_guard<std::mutex> lock(debug_mutex_);
        laser_points_base_ = laser_points_base;
        cluster_points_ = cluster_points;
        point_line_ = point_line;
    }else {
        laser_points_base_ = laser_points_base;
        cluster_points_ = cluster_points;
        point_line_ = point_line;
    }
}

void CarControlNode::updateFSM() {
    switch (current_state) {
        case SEARCHING_TARGET:
            break;
        case TRACING:
            break;
        case SEARCH_TABLE:
            break;
        case RETURNING_TABLE:

            break;

        default:
            break;
    }
}

void CarControlNode::changeState(RobotState new_state) {

}

/**
 * @brief 求直线外一点到直线距离
 * @param p 直线外一点
 * @param a 直线端点
 * @param b 直线端点
 * @return p点到直线ab的距离
 */
float CarControlNode::pointLineDistance(
    const cv::Point2f& p,
    const cv::Point2f& a,
    const cv::Point2f& b)
{
    cv::Point2f ab = b - a;
    cv::Point2f ap = p - a;

    float cross =
        std::abs(ab.x * ap.y - ab.y * ap.x);

    float norm =
        std::sqrt(ab.x * ab.x + ab.y * ab.y);

    if (norm < 1e-6f)
        return 0.0f;

    return cross / norm;
}

/**
 * @brief 求笛卡尔坐标系下的点与x轴负方向的角度
 * @param p 笛卡尔坐标系下的点
 * @return 角度（弧度）
 */
float CarControlNode::projectAndComputeAngle(const cv::Point3f &p) {
    return std::atan2(p.y, -p.x);
}

/**
 * @brief 2D点云聚类
 * @param points 输入点云
 * @param dist_thresh 聚类距离域值
 * @return 聚类后的点集
 */
std::vector<std::vector<cv::Point2f>>
 CarControlNode::clusterLaserPoints(
    const std::vector<cv::Point2f>& points,
    float threshold)
{
    std::vector<std::vector<cv::Point2f>> clusters;

    if (points.empty())
        return clusters;

    std::vector<cv::Point2f> current;

    current.push_back(points[0]);

    for (size_t i = 1; i < points.size(); i++)
    {
        float dist = cv::norm(points[i] - points[i - 1]);

        if (dist < threshold)
        {
            current.push_back(points[i]);
        }
        else
        {
            if (current.size() > 5)
                clusters.push_back(current);

            current.clear();
            current.push_back(points[i]);
        }
    }

    if (current.size() > 5)
        clusters.push_back(current);

    if (clusters.size() >= 2)
    {
        auto& first = clusters.front();
        auto& last  = clusters.back();

        float dist = cv::norm(first.front() - last.back());

        if (dist < threshold)
        {
            // 合并
            last.insert(
                last.end(),
                first.begin(),
                first.end());

            // 删除 first
            clusters.erase(clusters.begin());
        }
    }

    return clusters;
}

/**
 * @brief 点云拟合直线
 * @param cluster 聚类后的一类点云
 * @return 点云拟合直线后的结果
 */
LineFeature CarControlNode::fitLinePCA(
    const std::vector<cv::Point2f>& cluster)
{
    LineFeature line;

    Eigen::Vector2f mean(0,0);

    for (auto& p : cluster)
        mean += Eigen::Vector2f(p.x, p.y);

    mean /= cluster.size();

    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();

    for (auto& p : cluster)
    {
        Eigen::Vector2f d(p.x - mean.x(),
                          p.y - mean.y());

        cov += d * d.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f>
        solver(cov);

    line.normal =
        solver.eigenvectors().col(0).normalized();

    line.tangent =
        solver.eigenvectors().col(1).normalized();

    line.center = mean;

    line.length =
        (Eigen::Vector2f(
            cluster.front().x,
            cluster.front().y)
        -
        Eigen::Vector2f(
            cluster.back().x,
            cluster.back().y)).norm();
    line.valid = true;

    return line;
}

cv::Point2f CarControlNode::transformPointToYawWorld(const cv::Point2f &p, float yaw) {

    cv::Point2f pw;
    const float c = std::cos(-yaw);
    const float s = std::sin(-yaw);

    pw.x = c * p.x - s * p.y;
    pw.y = s * p.x + c * p.y;
    return pw;
}

std::vector<cv::Point2f> CarControlNode::transformPointsToYawWorld(
    const std::vector<cv::Point2f>& points,
    float yaw)
{
    std::vector<cv::Point2f> transformed;
    transformed.reserve(points.size());

    for (const auto& p : points)
    {
        cv::Point2f pw = transformPointToYawWorld(p, yaw);

        transformed.push_back(pw);
    }

    return transformed;
}

/**
 * @brief 极坐标转笛卡尔坐标系
 * @param range 长度
 * @param angle_rad 角度
 * @return  笛卡尔坐标系下的2D点
 */
cv::Point2f CarControlNode::polarToCartesian(float range, float angle_rad)
{
    return cv::Point2f(
        range * std::cos(angle_rad),
        range * std::sin(angle_rad));
}

std::vector<cv::Point2f> CarControlNode::laserScanToCartesian(
    const sensor_msgs::msg::LaserScan& scan)
{
    std::vector<cv::Point2f> points;
    points.reserve(scan.ranges.size());

    float angle = scan.angle_min;
    // float angle = now_car_yaw;
    for (const float range : scan.ranges) {
        if (std::isfinite(range) &&
            range >= scan.range_min &&
            range <= scan.range_max) {
            points.emplace_back(polarToCartesian(range, angle));
            }
        angle += scan.angle_increment;
    }

    return points;
}

/**
 * @brief 欧拉角转旋转矩阵
 * @param rpy
 * @return cv3x3矩阵
 */
cv::Matx33d CarControlNode::rpyToRotationMatrix(const cv::Vec3d& rpy)
{
    const double roll = rpy[0];
    const double pitch = rpy[1];
    const double yaw = rpy[2];

    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    const cv::Matx33d rotation_x(
        1.0, 0.0, 0.0,
        0.0, cr, -sr,
        0.0, sr, cr);

    const cv::Matx33d rotation_y(
        cp, 0.0, sp,
        0.0, 1.0, 0.0,
        -sp, 0.0, cp);

    const cv::Matx33d rotation_z(
        cy, -sy, 0.0,
        sy, cy, 0.0,
        0.0, 0.0, 1.0);

    return rotation_z * rotation_y * rotation_x;
}

/**
 *
 * @brief 三维点坐标系变换函数
 * @param point 需要转换的三维点
 * @param rotation 旋转矩阵
 * @param translation 平移矩阵
 * @return 按照旋转矩阵和平移矩阵转换后的三维点
 */
cv::Point3f CarControlNode::transformPoint(
    const cv::Point3f& point,
    const cv::Matx33d& rotation,
    const cv::Vec3d& translation)
{
    const cv::Vec3d point_vec(point.x, point.y, point.z);
    const cv::Vec3d transformed = rotation * point_vec + translation;

    return cv::Point3f(
        static_cast<float>(transformed[0]),
        static_cast<float>(transformed[1]),
        static_cast<float>(transformed[2]));
}

/**
 *
 * @param laser_points 等待转换的点
 * @return 转换后的雷达点集
 */
std::vector<cv::Point3f> CarControlNode::transformLaserPointsToBase(
    const std::vector<cv::Point2f>& laser_points) const
{
    std::vector<cv::Point3f> transformed_points;
    transformed_points.reserve(laser_points.size());

    for (const auto& point : laser_points) {
        transformed_points.push_back(transformPoint(
            cv::Point3f(point.x, point.y, 0.0F),
            lidar_to_base_rotation_,
            lidar_to_base_translation_));
    }

    return transformed_points;
}


/**
 * @brief 指定多线程核
 * @param cores 核ID
 */
void CarControlNode::bindThreadToCores(const std::vector<int>& cores)
{
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);

    for (int core : cores) {
        CPU_SET(core, &cpuset);
    }

    int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    if (ret != 0) {
        perror("pthread_setaffinity_np");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<CarControlNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
