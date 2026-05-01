#include "../include/car_control/tag_detector.hpp"

namespace {
    static void reorderCorners(std::vector<cv::Point2f>& pts)
    {
        // 计算中心
        cv::Point2f center(0,0);
        for (auto& p : pts) center += p;
        center *= 0.25f;

        // 按角度排序（逆时针）
        std::sort(pts.begin(), pts.end(),
            [&](const cv::Point2f& a, const cv::Point2f& b) {
                return atan2(a.y - center.y, a.x - center.x) <
                       atan2(b.y - center.y, b.x - center.x);
            });

        // 找到左上角作为起点
        int idx = 0;
        float min_sum = 1e9;
        for (int i = 0; i < 4; i++) {
            float s = pts[i].x + pts[i].y;
            if (s < min_sum) {
                min_sum = s;
                idx = i;
            }
        }

        // 旋转到左上为第一个
        std::rotate(pts.begin(), pts.begin() + idx, pts.end());
    }

cv::Point3f vectorToPoint3f(const cv::Mat& vector)
{
    if (vector.empty() || vector.total() != 3) {
        return cv::Point3f();
    }

    cv::Mat vector64;
    vector.convertTo(vector64, CV_64F);
    const cv::Mat flat = vector64.reshape(1, 3);

    return cv::Point3f(
        static_cast<float>(flat.at<double>(0, 0)),
        static_cast<float>(flat.at<double>(1, 0)),
        static_cast<float>(flat.at<double>(2, 0)));
}

}  // namespace

Detector::Detector(const CubeDetectParams& detector_param)
    : detector_params_(detector_param)
{
    const float half_len = detector_params_.tag_size_ / 2.0F;
    tag_def_point_.reserve(4);
    tag_def_point_.emplace_back(-half_len, half_len, 0.0F);
    tag_def_point_.emplace_back(half_len, half_len, 0.0F);
    tag_def_point_.emplace_back(half_len, -half_len, 0.0F);
    tag_def_point_.emplace_back(-half_len, -half_len, 0.0F);

    //apriltag 3 lib init
    tf_ = tag36h11_create();
    td_ = apriltag_detector_create();

    apriltag_detector_add_family(td_, tf_);

    td_->quad_decimate = 2.0;   // 降采样（>1更快，1最稳）
    td_->quad_sigma = 0.0;      // 高斯模糊
    td_->nthreads = std::max(1u, std::thread::hardware_concurrency() / 2);  // 多线程
    td_->debug = 0;
    td_->refine_edges = 1;
}
CubeDetectionResult Detector::detectCubes(cv::Mat& image)
{
    CubeDetectionResult result;
    if (image.empty()) {
        return result;
    }

    // 灰度图
    cv::Mat gray;
    if (image.channels() == 3) {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // 转成 AprilTag 格式
    image_u8_t img_header = {
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };

    // 检测tag
    zarray_t* detections = apriltag_detector_detect(td_, &img_header);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    int num = zarray_size(detections);
    marker_ids.reserve(num);
    marker_corners.reserve(num);

    for (int i = 0; i < num; i++) {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        marker_ids.push_back(det->id);

        std::vector<cv::Point2f> corners(4);
        for (int k = 0; k < 4; k++) {
            corners[k] = cv::Point2f(
                static_cast<float>(det->p[k][0]),
                static_cast<float>(det->p[k][1])
            );
        }
        //重新排列角点顺序
        reorderCorners(corners);
        marker_corners.push_back(corners);
    }


    // 释放 detection
    apriltag_detections_destroy(detections);

    // 按照ID分组
    const auto groups = groupById(marker_ids);

    result.cubes_camera.reserve(groups.size());
    result.cubes_base.reserve(groups.size());

    for (const auto& [id, indices] : groups) {
        // PNP解算并计算cube整体姿态
        CubeState cube = estimateCube(
            id,
            indices,
            marker_corners,
            tag_def_point_,
            detector_params_.camera_matrix_,
            detector_params_.dist_coeffs_);

        if (!cube.valid) {
            continue;
        }

        result.cubes_camera.push_back(cube);
        result.cubes_base.push_back(transformCubeToBase(cube));
    }

    return result;
}

std::unordered_map<int, std::vector<int>> Detector::groupById(
    const std::vector<int>& ids) const
{
    std::unordered_map<int, std::vector<int>> groups;
    for (size_t i = 0; i < ids.size(); ++i) {
        groups[ids[i]].push_back(static_cast<int>(i));
    }
    return groups;
}

CubeState Detector::estimateCube(
    int id,
    const std::vector<int>& indices,
    const std::vector<std::vector<cv::Point2f>>& corners,
    const std::vector<cv::Point3f>& tag_def_point,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs) const
{
    CubeState cube;
    cube.id = id;

    std::vector<cv::Point3f> centers;
    std::vector<cv::Mat> rvecs;
    centers.reserve(indices.size());
    rvecs.reserve(indices.size());
    cube.tags.reserve(indices.size());

    for (const int idx : indices) {
        if (idx < 0 || idx >= static_cast<int>(corners.size())) {
            continue;
        }
        if (corners[idx].size() != tag_def_point.size()) {
            continue;
        }

        cv::Mat rvec;
        cv::Mat tvec;
        if (!cv::solvePnP(
                tag_def_point,
                corners[idx],
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
                false,
                cv::SOLVEPNP_IPPE)) {
            continue;
        }

        TagPose tag;
        tag.id = id;
        tag.rvec = rvec.clone();
        tag.tvec = tvec.clone();

        cv::Mat rotation;
        cv::Rodrigues(tag.rvec, rotation);

        const double half_cube_size =
            static_cast<double>(detector_params_.cube_size_) / 2.0;
        const cv::Mat center_mat = tag.tvec - half_cube_size * rotation.col(2);
        tag.center = vectorToPoint3f(center_mat);

        centers.push_back(tag.center);
        rvecs.push_back(tag.rvec);
        cube.tags.push_back(std::move(tag));
    }

    if (centers.empty()) {
        return cube;
    }

    cv::Point3f avg(0.0F, 0.0F, 0.0F);
    for (const auto& center : centers) {
        avg += center;
    }
    avg *= static_cast<float>(1.0 / static_cast<double>(centers.size()));

    cube.center = avg;
    cube.tvec = (cv::Mat_<double>(3, 1) << avg.x, avg.y, avg.z);
    cube.rvec = rvecs.front().clone();
    cv::Rodrigues(cube.rvec, cube.R);
    cube.valid = true;
    return cube;
}

cv::Point3f Detector::transformPoint(
    const cv::Point3f& point,
    const cv::Matx33d& rotation,
    const cv::Vec3d& translation) const
{
    const cv::Vec3d point_vec(point.x, point.y, point.z);
    const cv::Vec3d transformed = rotation * point_vec + translation;

    return cv::Point3f(
        static_cast<float>(transformed[0]),
        static_cast<float>(transformed[1]),
        static_cast<float>(transformed[2]));
}

CubeState Detector::transformCubeToBase(const CubeState& cube) const
{
    CubeState transformed_cube;
    transformed_cube.id = cube.id;
    transformed_cube.valid = cube.valid;
    transformed_cube.center = transformPoint(
        cube.center,
        detector_params_.camera_to_base_rotation_,
        detector_params_.camera_to_base_translation_);
    transformed_cube.tvec = (cv::Mat_<double>(3, 1) <<
        transformed_cube.center.x,
        transformed_cube.center.y,
        transformed_cube.center.z);

    if (!cube.R.empty()) {
        transformed_cube.R =
            cv::Mat(detector_params_.camera_to_base_rotation_) * cube.R;
        cv::Rodrigues(transformed_cube.R, transformed_cube.rvec);
    }

    transformed_cube.tags.reserve(cube.tags.size());
    for (const auto& source_tag : cube.tags) {
        TagPose transformed_tag;
        transformed_tag.id = source_tag.id;
        transformed_tag.center = transformPoint(
            source_tag.center,
            detector_params_.camera_to_base_rotation_,
            detector_params_.camera_to_base_translation_);

        const cv::Point3f tag_position = source_tag.tvec.empty()
            ? transformed_tag.center
            : transformPoint(
                vectorToPoint3f(source_tag.tvec),
                detector_params_.camera_to_base_rotation_,
                detector_params_.camera_to_base_translation_);
        transformed_tag.tvec = (cv::Mat_<double>(3, 1) <<
            tag_position.x,
            tag_position.y,
            tag_position.z);

        if (!source_tag.rvec.empty()) {
            cv::Mat tag_rotation;
            cv::Rodrigues(source_tag.rvec, tag_rotation);
            tag_rotation =
                cv::Mat(detector_params_.camera_to_base_rotation_) *
                tag_rotation;
            cv::Rodrigues(tag_rotation, transformed_tag.rvec);
        }

        transformed_cube.tags.push_back(std::move(transformed_tag));
    }

    return transformed_cube;
}
