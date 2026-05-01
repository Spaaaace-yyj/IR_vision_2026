#ifndef TAG_DETECTOR_HPP
#define TAG_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <unordered_map>
#include <vector>
#include <thread>
#include <pthread.h>
#include <sched.h>

#include <apriltag.h>
#include <tag36h11.h>

struct TagPose {
    int id = -1;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Point3f center;
};

struct CubeState {
    int id = -1;
    std::vector<TagPose> tags;

    cv::Point3f center;
    cv::Mat R;

    cv::Mat rvec;
    cv::Mat tvec;

    bool valid = false;
};

struct CubeDetectParams {

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv::Matx33d camera_to_base_rotation_ = cv::Matx33d::eye();
    cv::Vec3d camera_to_base_translation_ = cv::Vec3d(0.0, 0.0, 0.0);

    float cube_size_;
    float tag_size_;
};

struct CubeDetectionResult {
    std::vector<CubeState> cubes_camera;
    std::vector<CubeState> cubes_base;
};

class Detector {
public:
    explicit Detector(const CubeDetectParams& detector_params);

    CubeDetectionResult detectCubes(cv::Mat& image);

    CubeState estimateCube(
        int id,
        const std::vector<int>& indices,
        const std::vector<std::vector<cv::Point2f>>& corners,
        const std::vector<cv::Point3f>& tag_def_point,
        const cv::Mat& camera_matrix,
        const cv::Mat& dist_coeffs) const;

    CubeState transformCubeToBase(const CubeState& cube) const;

    cv::Point3f transformPoint(
        const cv::Point3f& point,
        const cv::Matx33d& rotation,
        const cv::Vec3d& translation) const;

    ~Detector() = default;

private:
    std::unordered_map<int, std::vector<int>> groupById(
        const std::vector<int>& ids) const;

    std::vector<cv::Point3f> tag_def_point_;
    CubeDetectParams detector_params_;

    apriltag_family_t* tf_;
    apriltag_detector_t* td_;
};

#endif
