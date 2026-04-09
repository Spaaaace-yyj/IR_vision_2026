#include "../include/car_control/car_control_node.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

CarControlNode::CarControlNode() : Node("car_control_node")
{
    //打开摄像头
    // cap_.open(1, cv::CAP_V4L2);
    cap_.open(1);
    if(!cap_.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Cannot open camera");
        return; 
    }

    image_timer_ = this->create_wall_timer(33ms, std::bind(&CarControlNode::imageCallback, this));
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/car_control/image_debug", rclcpp::SensorDataQoS());
    scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::SensorDataQoS(),
        std::bind(&CarControlNode::scanCallback, this, std::placeholders::_1));

    detector_dict_ = cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11));    
    detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();

    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    show_thread_ = std::thread(&CarControlNode::debugThread, this);
    
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

    float half_len = tag_size / 2;
    tag_def_point.emplace_back(cv::Point3f(-half_len, half_len, 0));
    tag_def_point.emplace_back(cv::Point3f(half_len, half_len, 0));
    tag_def_point.emplace_back(cv::Point3f(half_len, -half_len, 0));
    tag_def_point.emplace_back(cv::Point3f(-half_len, -half_len, 0));

    camera_matrix_ = (cv::Mat_<double>(3,3) << 
        504.504779, 0, 647.473284,
        0, 502.266322, 363.157317,
        0, 0, 1);

    dist_coeffs_ = (cv::Mat_<double>(1,5) << 0.003130, 0.002283, 0.001391, 0.001827, 0.0);

    RCLCPP_INFO(this->get_logger(), "Success Init Car Control node!");
}

void CarControlNode::imageCallback()
{
    static int fps = 0;
    static auto start_time = this->now();
    auto t0 = this->now();

    if(this->now() - start_time >= rclcpp::Duration::from_seconds(1.0)){
        RCLCPP_INFO(this->get_logger(), "FPS: %d", fps);
        fps = 0;
        start_time = this->now();
    }
    fps++;

    cv::Mat frame;
    cap_.grab();
    cap_.retrieve(frame);

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty frame");
        return;
    }

    cv::Mat gray, bin;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    //全画面识别
    cv::aruco::detectMarkers(
        frame,
        detector_dict_,
        marker_corners,
        marker_ids,
        detector_params_);
    
    //筛选干扰框
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(bin, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> candidate_rois;

    for (size_t i = 0; i < contours.size(); i++)
    {
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contours[i], approx,
            0.02 * cv::arcLength(contours[i], true), true);
        //筛选矩形边框
        if (approx.size() != 4) continue;
        if (!cv::isContourConvex(approx)) continue;
        
        double area = cv::contourArea(approx);
        if (area < 500) continue;
        
        //除去没有子边框
        int child = hierarchy[i][2];
        if (child == -1) continue;
        
        std::vector<cv::Point> approx_child;
        cv::approxPolyDP(contours[child], approx_child,
            0.02 * cv::arcLength(contours[child], true), true);

        if (approx_child.size() != 4) continue;

        double area_child = cv::contourArea(approx_child);
        if (area_child < 600) continue;

        cv::Rect roi = cv::boundingRect(approx);
        roi &= cv::Rect(0, 0, frame.cols, frame.rows);

        candidate_rois.push_back(roi);

        // debug ROI
        // cv::rectangle(frame, roi, cv::Scalar(200, 0, 200), 1);
    }
    //Roi内识别
    for (auto &roi : candidate_rois)
    {
        cv::Mat sub = frame(roi);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(
            sub,
            detector_dict_,
            corners,
            ids,
            detector_params_);

        if (ids.empty()) continue;

        for (size_t i = 0; i < corners.size(); i++)
        {
            // 坐标映射回原图
            for (auto &pt : corners[i]) {
                pt.x += roi.x;
                pt.y += roi.y;
            }

            bool keep = true;
            cv::Rect new_rect = cv::boundingRect(corners[i]);

            for (size_t j = 0; j < marker_corners.size(); j++)
            {
                cv::Rect old_rect = cv::boundingRect(marker_corners[j]);

                float inter = (new_rect & old_rect).area();
                float uni = new_rect.area() + old_rect.area() - inter;

                float iou = inter / uni;

                if (iou > 0.3)
                {
                    if (new_rect.area() > old_rect.area()) {
                        marker_ids[j] = ids[i];
                        marker_corners[j] = corners[i];
                    }
                    keep = false;
                    break;
                }
            }

            if (keep) {
                marker_ids.push_back(ids[i]);
                marker_corners.push_back(corners[i]);
            }
        }
    }
    //debug
    if (!marker_ids.empty()) {
        // cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
        for(size_t i = 0; i < marker_ids.size(); i++)
        {
            cv::line(frame, marker_corners[i][0], marker_corners[i][1], cv::Scalar(0,255,0), 1);
            cv::line(frame, marker_corners[i][1], marker_corners[i][2], cv::Scalar(0,255,0), 1);
            cv::line(frame, marker_corners[i][2], marker_corners[i][3], cv::Scalar(0,255,0), 1);
            cv::line(frame, marker_corners[i][3], marker_corners[i][0], cv::Scalar(0,255,0), 1);
        }
    }

    auto groups = groupById(marker_ids);

    std::vector<CubeState> cubes;
    std::vector<CubeState> cubes_base;
    cubes_base.reserve(groups.size());

    for (auto &[id, indices] : groups)
    {
        CubeState cube = estimateCube(
            id,
            indices,
            marker_corners,
            tag_def_point,
            camera_matrix_,
            dist_coeffs_);

        if (cube.valid) {
            cubes.push_back(cube);
            //转换到底盘坐标系
            cubes_base.push_back(transformCubeToBase(cube));
        }
    }

    double latency = (this->now() - t0).seconds() * 1000.0;

    std::stringstream ss;
    ss << "Latency: " << std::fixed << std::setprecision(2) << latency << " ms";

    cv::putText(frame, ss.str(),
        cv::Point(10, 30),
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        cv::Scalar(0,255,0),
        2);

    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        latest_frame_ = frame.clone();
        cubes_ = cubes;
        cubes_base_ = cubes_base;
    }
}

void CarControlNode::debugThread()
{
    while (rclcpp::ok())
    {
        cv::Mat img;
        std::vector<CubeState> cubes;
        std::vector<CubeState> cubes_base;
        std::vector<cv::Point3f> laser_points_base;

        {
            std::lock_guard<std::mutex> lock(img_mutex_);

            if (!latest_frame_.empty()) {
                img = latest_frame_.clone();
            }
            cubes = cubes_;
            cubes_base = cubes_base_;
            laser_points_base = laser_points_base_;
        }

        if (!img.empty()) {
            int image_width = img.cols;
            int image_height = img.rows;
            drawCubes(img, cubes, camera_matrix_, dist_coeffs_);
            cv::line(img, cv::Point(image_width / 2, 0), cv::Point(image_width / 2, image_height), cv::Scalar(255,255,255), 1);
            cv::line(img, cv::Point(0, image_height / 2), cv::Point(image_width, image_height / 2), cv::Scalar(255,255,255), 1);
            cv::circle(img, cv::Point(image_width / 2, image_height / 2), 15, cv::Scalar(100,255,100), 1);
            cv::imshow("Camera", img);
        }

        cv::Mat laser_scan_view = buildLaserScanView(laser_points_base, cubes_base);
        cv::imshow("LaserScan", laser_scan_view);
        cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

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

void CarControlNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const std::vector<cv::Point2f> laser_points = laserScanToCartesian(*msg);
    //转换到底盘坐标系
    const std::vector<cv::Point3f> laser_points_base =
        transformLaserPointsToBase(laser_points);

    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        laser_points_base_ = laser_points_base;
    }
}

std::unordered_map<int, std::vector<int>> CarControlNode::groupById(
    const std::vector<int>& ids)
{
    std::unordered_map<int, std::vector<int>> groups;
    for (size_t i = 0; i < ids.size(); i++) {
        groups[ids[i]].push_back(i);
    }
    return groups;
}

CubeState CarControlNode::estimateCube(
    int id,
    const std::vector<int>& indices,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<cv::Point3f>& tag_def_point,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs)
{
    CubeState cube;
    cube.id = id;

    std::vector<cv::Point3f> centers;
    std::vector<cv::Mat> rvecs;

    for (auto idx : indices) {
        cv::Mat rvec, tvec;

        if (!cv::solvePnP(tag_def_point, corners[idx], camera_matrix, dist_coeffs, rvec, tvec, false, cv::SOLVEPNP_IPPE))
            continue;

        TagPose tag;
        tag.id = id;
        tag.rvec = rvec;
        tag.tvec = tvec;

        cv::Mat R;
        cv::Rodrigues(rvec, R);

        double d = cube_size / 2;  // 立方体半尺寸
        cv::Mat center_mat = tvec - d * R.col(2);

        tag.center = cv::Point3f(
            center_mat.at<double>(0),
            center_mat.at<double>(1),
            center_mat.at<double>(2));

        centers.push_back(tag.center);
        rvecs.push_back(rvec);
        cube.tags.push_back(tag);
    }

    if (centers.empty()) return cube;

    // 平均中心
    cv::Point3f avg(0,0,0);
    for (auto &c : centers) avg += c;
    avg *= (1.0 / centers.size());

    cube.center = avg;
    cube.tvec = (cv::Mat_<double>(3,1) << avg.x, avg.y, avg.z);
    cube.rvec = rvecs[0];

    // 暂时用第一个姿态
    cv::Rodrigues(rvecs[0], cube.R);

    cube.valid = true;
    return cube;
}

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

CubeState CarControlNode::transformCubeToBase(const CubeState& cube) const
{
    CubeState transformed_cube;
    transformed_cube.id = cube.id;
    transformed_cube.valid = cube.valid;
    transformed_cube.center = transformPoint(
        cube.center,
        camera_to_base_rotation_,
        camera_to_base_translation_);

    transformed_cube.tvec = (cv::Mat_<double>(3, 1) <<
        transformed_cube.center.x,
        transformed_cube.center.y,
        transformed_cube.center.z);

    if (!cube.R.empty()) {
        transformed_cube.R = cv::Mat(camera_to_base_rotation_) * cube.R;
        cv::Rodrigues(transformed_cube.R, transformed_cube.rvec);
    }

    transformed_cube.tags.reserve(cube.tags.size());
    for (const auto& source_tag : cube.tags) {
        TagPose transformed_tag;
        transformed_tag.id = source_tag.id;
        transformed_tag.center = transformPoint(
            source_tag.center,
            camera_to_base_rotation_,
            camera_to_base_translation_);

        transformed_tag.tvec = (cv::Mat_<double>(3, 1) <<
            transformed_tag.center.x,
            transformed_tag.center.y,
            transformed_tag.center.z);

        if (!source_tag.rvec.empty()) {
            cv::Mat tag_rotation;
            cv::Rodrigues(source_tag.rvec, tag_rotation);
            tag_rotation = cv::Mat(camera_to_base_rotation_) * tag_rotation;
            cv::Rodrigues(tag_rotation, transformed_tag.rvec);
        }

        transformed_cube.tags.push_back(std::move(transformed_tag));
    }

    return transformed_cube;
}

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

        for (auto &tag : cube.tags)
        {
            cv::drawFrameAxes(frame,
                camera_matrix,
                dist_coeffs,
                tag.rvec,
                tag.tvec,
                0.04);
        }
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
  auto node = std::make_shared<CarControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
