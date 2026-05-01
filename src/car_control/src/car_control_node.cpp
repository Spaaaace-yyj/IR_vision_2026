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

    int ret = pthread_setaffinity_np(pthread_self(),
                                     sizeof(cpu_set_t),
                                     &cpuset);

    if (ret != 0) {
        perror("pthread_setaffinity_np");
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
    // RCLCPP_INFO(this->get_logger(), "target_yaw = %f | is_tracing = %f", cmd_msg.target_yaw, is_tracing_);

}

/**
 *
 * @brief debug线程，绘制debug数据
 */
void CarControlNode::debugThread()
{
    bindThreadToCores({7});
    cv::Mat img;
    std::vector<CubeState> cubes;
    std::vector<CubeState> cubes_base;
    std::vector<cv::Point3f> laser_points_base;
    double detector_latency;
    while (debug_running_.load(std::memory_order_relaxed) && rclcpp::ok())
    {
        static auto last = this->now();

        if ((this->now() - last).seconds() < 0.05) { // 20Hz
            continue;
        }
        last = this->now();

        uint16_t cur = debug_frame_id_.load(std::memory_order_relaxed);
        if (cur == last_debug_processed_id_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        last_debug_processed_id_.store(cur, std::memory_order_relaxed);
        int r = debug_read_index_.load(std::memory_order_acquire);
        DebugPacket pkt = debug_buffer[r];
        if(debug_mode_){
            img = pkt.frame;
            cubes = pkt.result.cubes_camera;
            cubes_base = pkt.result.cubes_base;
            laser_points_base = pkt.laser_points;
            detector_latency = pkt.detector_latency;

            if (!img.empty()) {
                int image_width = img.cols;
                int image_height = img.rows;
                drawCubes(img, cubes, camera_matrix_, dist_coeffs_);
                cv::line(img, cv::Point(image_width / 2, 0), cv::Point(image_width / 2, image_height), cv::Scalar(255,255,255), 1);
                cv::line(img, cv::Point(0, image_height / 2), cv::Point(image_width, image_height / 2), cv::Scalar(255,255,255), 1);
                cv::circle(img, cv::Point(image_width / 2, image_height / 2), 15, cv::Scalar(100,255,100), 1);
                std::stringstream ss;
                ss << "Latency: " << std::fixed << std::setprecision(2) << detector_latency << " ms";

                cv::putText(img, ss.str(),
                    cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    cv::Scalar(0,255,0),
                    2);
                // cv::imshow("Camera", img);
            }
        
            cv::Mat laser_scan_view = buildLaserScanView(laser_points_base, cubes_base);
            // cv::imshow("LaserScan", laser_scan_view);
            // cv::waitKey(1);

            std_msgs::msg::Header header;
            header.stamp = this->now();

            if (!img.empty()) {
                cv_bridge::CvImage debug_img(header, "bgr8", img);
                auto image_msg = debug_img.toImageMsg();
                image_publisher_->publish(*image_msg);
            }
            
            cv_bridge::CvImage scan_img(header, "bgr8", laser_scan_view);
            auto scan_msg = scan_img.toImageMsg();
            scan_image_publisher_->publish(*scan_msg);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

/**
 *
 * @brief 雷达消息回调函数
 * @param msg
 */
void CarControlNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const std::vector<cv::Point2f> laser_points = laserScanToCartesian(*msg);
    //转换到底盘坐标系
    const std::vector<cv::Point3f> laser_points_base =
        transformLaserPointsToBase(laser_points);

    {
        std::lock_guard<std::mutex> lock(debug_mutex_);
        laser_points_base_ = laser_points_base;
    }
}

float CarControlNode::projectAndComputeAngle(const cv::Point3f &p) {
    return std::atan2(p.y, -p.x);
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
 *
 * @brief 绘制x标在图像上
 * @param p 绘制坐标
 * @param frame 需要绘制的图像
 */
void drawX(cv::Point2f p, cv::Mat& frame){
    cv::line(frame, p + cv::Point2f(-30, -30), p + cv::Point2f(30, 30), cv::Scalar(0,0,255), 1);
    cv::line(frame, p + cv::Point2f(-30, 30), p + cv::Point2f(30, -30), cv::Scalar(0,0,255), 1);
}

void CarControlNode::drawCubes(
    cv::Mat& frame,
    const std::vector<CubeState>& cubes,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs)
{
    if(cubes.empty()){
        cv::putText(frame, "No cubes detected",
            cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0,0,255),
            2);
        drawX(cv::Point2f(frame.cols / 2, frame.rows / 2), frame);
        return;
    }else{
        std::stringstream ss;
        ss << "Cubes detected num: " << cubes.size();

        cv::putText(frame, ss.str(),
            cv::Point(10, 60),
            cv::FONT_HERSHEY_SIMPLEX,
            1.0,
            cv::Scalar(0,255,0),
            2);
    }

    std::vector<cv::Point2f> target_center2d;

    for (const auto& cube : cubes)
    {
        if (!cube.valid) continue;

        std::vector<cv::Point3f> center3d = {cube.center};
        std::vector<cv::Point2f> center2d;

        cv::projectPoints(center3d,
            cv::Mat::zeros(3,1,CV_64F),
            cv::Mat::zeros(3,1,CV_64F),
            camera_matrix, dist_coeffs,
            center2d);

        if (!center2d.empty()) {
            cv::circle(frame, center2d[0], 3, cv::Scalar(100,255,255), -1);
            cv::line(frame, center2d[0], cv::Point(frame.cols / 2, frame.rows), cv::Scalar(255,255,255), 1);

            cv::putText(frame,
                "ID:" + std::to_string(cube.id),
                center2d[0],
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0,0,255),
                1);
        }

        if (cube.id == 1 && is_blue) {
            target_center2d = center2d;
        }
        if (cube.id == 2 && !is_blue) {
            target_center2d = center2d;
        }
        if (cube.id == 0) {
            target_center2d = center2d;
        }

        if(cube.id == 0 && !center2d.empty()){
            drawX(center2d[0], frame);
        }
        if (cube.R.empty()) continue;

        float l = cube_size / 2.0;

        std::vector<cv::Point3f> cube_pts_local = {
            {-l,-l,-l},{ l,-l,-l},{ l, l,-l},{-l, l,-l},
            {-l,-l, l},{ l,-l, l},{ l, l, l},{-l, l, l}
        };

        std::vector<cv::Point3f> cube_pts_world;

        for (auto &p : cube_pts_local)
        {
            cv::Mat pt = (cv::Mat_<double>(3,1) << p.x, p.y, p.z);

            cv::Mat transformed = cube.R * pt +
                (cv::Mat_<double>(3,1) <<
                    cube.center.x,
                    cube.center.y,
                    cube.center.z);

            cube_pts_world.emplace_back(
                transformed.at<double>(0),
                transformed.at<double>(1),
                transformed.at<double>(2));
        }

        std::vector<cv::Point2f> pts2d;

        cv::projectPoints(
            cube_pts_world,
            cv::Mat::zeros(3,1,CV_64F),
            cv::Mat::zeros(3,1,CV_64F),
            camera_matrix,
            dist_coeffs,
            pts2d);

        if (pts2d.size() != 8) continue;

        std::vector<std::pair<int,int>> edges = {
            {0,1},{1,2},{2,3},{3,0},
            {4,5},{5,6},{6,7},{7,4},
            {0,4},{1,5},{2,6},{3,7},
        };

        cv::Scalar edge_color;
        if(cube.id == 1){
            edge_color = cv::Scalar(255, 255, 0);
        }else if(cube.id == 2){
            edge_color = cv::Scalar(0, 255, 255);
        }else{
            edge_color = cv::Scalar(200, 255, 255);
        }

        for (auto &e : edges)
        {
            cv::line(frame,
                pts2d[e.first],
                pts2d[e.second],
                edge_color,
                1);
        }

        // for (auto &tag : cube.tags)
        // {
        //     cv::drawFrameAxes(frame,
        //         camera_matrix,
        //         dist_coeffs,
        //         tag.rvec,
        //         tag.tvec,
        //         0.04);
        // }
    }
    if (!target_center2d.empty()) {
        drawX(target_center2d[0], frame);
    }
}

//debug
cv::Mat CarControlNode::buildLaserScanView(
    const std::vector<cv::Point3f>& points,
    const std::vector<CubeState>& cubes,
    int image_size)
{
    constexpr int kMargin = 30;
    cv::Mat view(image_size, image_size, CV_8UC3, cv::Scalar(30, 30, 30));
    int valid_cube_count = 0;

    const cv::Point center(image_size / 2, image_size / 2);
    cv::line(view, cv::Point(center.x, 0), cv::Point(center.x, image_size), cv::Scalar(80, 80, 80), 1);
    cv::line(view, cv::Point(0, center.y), cv::Point(image_size, center.y), cv::Scalar(80, 80, 80), 1);
    cv::circle(view, center, 4, cv::Scalar(0, 255, 255), -1);

    float max_extent = 0.1F;
    for (const auto& point : points) {
        max_extent = std::max(max_extent, std::abs(point.x));
        max_extent = std::max(max_extent, std::abs(point.y));
    }
    for (const auto& cube : cubes) {
        if (!cube.valid) {
            continue;
        }

        ++valid_cube_count;
        max_extent = std::max(max_extent, std::abs(cube.center.x));
        max_extent = std::max(max_extent, std::abs(cube.center.y));
    }

    const float usable_radius = static_cast<float>(std::max(1, image_size / 2 - kMargin));
    const float scale = usable_radius / max_extent;

    for (const auto& point : points) {
        const int px = static_cast<int>(std::lround(center.x - point.y * scale));
        const int py = static_cast<int>(std::lround(center.y + point.x * scale));

        if (px >= 0 && px < image_size && py >= 0 && py < image_size) {
            cv::circle(view, cv::Point(px, py), 1, cv::Scalar(0, 255, 0), -1);
        }
    }

    for (const auto& cube : cubes) {
        if (!cube.valid) {
            continue;
        }

        const int px = static_cast<int>(std::lround(center.x - cube.center.y * scale));
        const int py = static_cast<int>(std::lround(center.y + cube.center.x * scale));

        if (px < 0 || px >= image_size || py < 0 || py >= image_size) {
            continue;
        }

        const cv::Point cube_center(px, py);
        cv::circle(view, cube_center, 2, cv::Scalar(0, 0, 255), -1);
        cv::circle(view, cube_center, 3, cv::Scalar(0, 255, 255), 1);

        std::ostringstream label_stream;
        label_stream << std::fixed << std::setprecision(2)
                     << "ID:" << cube.id
                     << " (" << cube.center.x
                     << ", " << cube.center.y
                     << ", " << cube.center.z << ")";

        const int label_x = std::max(10, std::min(px + 12, image_size - 220));
        const int label_y = std::max(20, std::min(py - 12, image_size - 10));
        const cv::Point label_origin(
            label_x,
            label_y);

        cv::putText(
            view,
            label_stream.str(),
            label_origin,
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            cv::Scalar(0, 220, 255),
            1);
    }

    cv::putText(view, "FWD (-X)", cv::Point(center.x - 36, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    cv::putText(view, "LEFT (+Y)", cv::Point(10, center.y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200, 200, 200), 1);
    cv::putText(view, "REAR (+X)", cv::Point(center.x - 40, image_size - 75), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(160, 160, 160), 1);
    cv::putText(view, "RIGHT (-Y)", cv::Point(image_size - 92, center.y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(160, 160, 160), 1);
    cv::putText(
        view,
        "Points: " + std::to_string(points.size()),
        cv::Point(10, image_size - 30),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(255, 255, 255),
        1);
    cv::putText(
        view,
        "Cubes: " + std::to_string(valid_cube_count),
        cv::Point(10, image_size - 50),
        cv::FONT_HERSHEY_SIMPLEX,
        0.6,
        cv::Scalar(0, 220, 255),
        1);
    cv::putText(
        view,
        "Scale: " + std::to_string(scale).substr(0, 5) + " px/m",
        cv::Point(10, image_size - 10),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 255, 255),
        1);

    return view;
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
