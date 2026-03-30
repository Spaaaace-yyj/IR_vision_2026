#include "../include/car_control/car_control_node.hpp"

CarControlNode::CarControlNode() : Node("car_control_node")
{
    //打开摄像头
    cap_.open(1, cv::CAP_V4L2);
    if(!cap_.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Cannot open camera");
        return; 
    }

    image_timer_ = this->create_wall_timer(33ms, std::bind(&CarControlNode::imageCallback, this));
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/car_control/image_debug", rclcpp::SensorDataQoS());

    detector_dict_ = cv::makePtr<cv::aruco::Dictionary>(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11));    
    detector_params_ = cv::makePtr<cv::aruco::DetectorParameters>();

    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

    show_thread_ = std::thread(&CarControlNode::debugThread, this);
    
    tag_size = this->declare_parameter("tag_size", 0.08);
    cube_size = this->declare_parameter("cube_size", 0.15);

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
        cv::Mat sub = gray(roi);

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
        cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
    }

    auto groups = groupById(marker_ids);

    std::vector<CubeState> cubes;

    for (auto &[id, indices] : groups)
    {
        CubeState cube = estimateCube(
            id,
            indices,
            marker_corners,
            tag_def_point,
            camera_matrix_,
            dist_coeffs_);

        if (cube.valid)
            cubes.push_back(cube);
    }

    drawCubes(frame, cubes, camera_matrix_, dist_coeffs_);

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
    }
}

void CarControlNode::debugThread()
{
    while (rclcpp::ok())
    {
        cv::Mat img;

        {
            std::lock_guard<std::mutex> lock(img_mutex_);

            if (latest_frame_.empty())
                continue;

            img = latest_frame_.clone();
        }

        cv::imshow("Camera", img);
        cv::waitKey(1);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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

    // 暂时用第一个姿态
    cv::Rodrigues(rvecs[0], cube.R);

    cube.valid = true;
    return cube;
}

void CarControlNode::drawCubes(
    cv::Mat& frame,
    const std::vector<CubeState>& cubes,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs)
{
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
            cv::circle(frame, center2d[0], 6, cv::Scalar(0,255,255), -1);

            cv::putText(frame,
                "ID:" + std::to_string(cube.id),
                center2d[0],
                cv::FONT_HERSHEY_SIMPLEX,
                0.6,
                cv::Scalar(0,255,255),
                2);
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
            {0,4},{1,5},{2,6},{3,7}
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
                2);
        }

        for (auto &tag : cube.tags)
        {
            cv::drawFrameAxes(frame,
                camera_matrix,
                dist_coeffs,
                tag.rvec,
                tag.tvec,
                0.05);
        }
    }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
