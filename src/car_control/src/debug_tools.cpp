//
// Created by nuc on 2026/5/7.
//

#include "../include/car_control/car_control_node.hpp"


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
    std::vector<std::vector<cv::Point2f>> cluster_points;
    std::vector<LineFeature> lines;
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
        {
            std::lock_guard<std::mutex> lock(debug_mutex_);
            cluster_points = cluster_points_;
            lines = point_line_;
        }


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

            cv::Mat laser_scan_view = buildLaserScanView(laser_points_base, cluster_points, lines, cubes_base);
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
 * @brief 绘制x标在图像上
 * @param p 绘制坐标
 * @param frame 需要绘制的图像
 */
void CarControlNode::drawX(cv::Point2f p, cv::Mat& frame){
    cv::line(frame, p + cv::Point2f(-30, -30), p + cv::Point2f(30, 30), cv::Scalar(0,0,255), 1);
    cv::line(frame, p + cv::Point2f(-30, 30), p + cv::Point2f(30, -30), cv::Scalar(0,0,255), 1);
}

/**
 * @brief 构造雷达点云debug图像
 * @param points 雷达点云
 * @param clusters 聚类后点云
 * @param lines 拟合直线
 * @param cubes 方块解算结果
 * @param image_size 返回图像尺寸
 * @return 构造后的mat
 */
cv::Mat CarControlNode::buildLaserScanView(
    const std::vector<cv::Point3f>& points,
    const std::vector<std::vector<cv::Point2f>>& clusters,
    const std::vector<LineFeature>& lines,
    const std::vector<CubeState>& cubes,
    int image_size)
{
    //debug laserscan color list
    std::vector<cv::Scalar> cluster_colors = {
        {255, 0, 0},
        {0, 255, 0},
        {0, 0, 255},
        {255, 255, 0},
        {255, 0, 255},
        {0, 255, 255},
        {200, 100, 255},
        {100, 255, 200}
    };

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

    // for (const auto& point : points) {
    //     const int px = static_cast<int>(std::lround(center.x - point.y * scale));
    //     const int py = static_cast<int>(std::lround(center.y + point.x * scale));
    //
    //     if (px >= 0 && px < image_size && py >= 0 && py < image_size) {
    //         cv::circle(view, cv::Point(px, py), 1, cv::Scalar(0, 255, 0), -1);
    //     }
    // }
    for (size_t cid = 0; cid < clusters.size(); ++cid)
    {
        const auto& cluster = clusters[cid];

        cv::Scalar color =
            cluster_colors[cid % cluster_colors.size()];

        for (const auto& point : cluster)
        {
            int px = static_cast<int>(
                std::lround(center.x - point.y * scale));

            int py = static_cast<int>(
                std::lround(center.y + point.x * scale));

            if (px >= 0 && px < image_size &&
                py >= 0 && py < image_size)
            {
                cv::circle(
                    view,
                    cv::Point(px, py),
                    2,
                    color,
                    -1);
            }
        }
    }

    for (const auto& line : lines)
    {
        if (!line.valid)
            continue;

        Eigen::Vector2f p1 =
            line.center -
            line.tangent * line.length * 0.5f;

        Eigen::Vector2f p2 =
            line.center +
            line.tangent * line.length * 0.5f;

        cv::Point cv_p1(
            static_cast<int>(
                std::lround(center.x - p1.y() * scale)),
            static_cast<int>(
                std::lround(center.y + p1.x() * scale)));

        cv::Point cv_p2(
            static_cast<int>(
                std::lround(center.x - p2.y() * scale)),
            static_cast<int>(
                std::lround(center.y + p2.x() * scale)));

        cv::line(
            view,
            cv_p1,
            cv_p2,
            cv::Scalar(255, 255, 255),
            2);
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

/**
 * @brief 构造cube识别结果可视化图像
 * @param frame 需要绘制的图像引用
 * @param cubes 方块识别结果
 * @param camera_matrix 相机内参矩阵
 * @param dist_coeffs 相机畸变参数
 */
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
