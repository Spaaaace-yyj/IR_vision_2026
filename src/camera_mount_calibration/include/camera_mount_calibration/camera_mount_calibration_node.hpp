#ifndef CAMERA_MOUNT_CALIBRATION_NODE_HPP_
#define CAMERA_MOUNT_CALIBRATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <string>
#include <vector>

class CameraMountCalibrationNode : public rclcpp::Node
{
public:
  CameraMountCalibrationNode();
  ~CameraMountCalibrationNode() override;

private:
  void timerCallback();

  bool detectChessboard(
    const cv::Mat & gray_image,
    std::vector<cv::Point2f> & corners);

  bool detectChessboardScaled(
    const cv::Mat & gray_image,
    std::vector<cv::Point2f> & corners) const;

  void refineCorners(
    const cv::Mat & gray_image,
    std::vector<cv::Point2f> & corners) const;

  void updateTrackingState(const std::vector<cv::Point2f> & corners);

  static cv::Rect expandRect(
    const cv::Rect & rect,
    const cv::Size & image_size,
    float padding_ratio);

  static std::vector<cv::Point3f> createBoardObjectPoints(
    const cv::Size & board_size,
    double square_size);

  static cv::Matx33d opticalFrameToMountFrameRotation();

  static cv::Matx33d rodriguesToMatrix(const cv::Mat & rvec);

  static cv::Vec3d rotationMatrixToRpy(const cv::Matx33d & rotation);

  static cv::Vec3d matToVec3d(const cv::Mat & mat);

  static std::string formatRpyDegrees(const cv::Vec3d & rpy_rad);

  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  cv::Size board_size_;
  std::vector<cv::Point3f> board_object_points_;
  cv::Mat last_rvec_;
  cv::Mat last_tvec_;
  std::vector<cv::Point2f> last_corners_;
  cv::Rect last_board_roi_;

  int camera_id_ = 0;
  int frame_width_ = 1280;
  int frame_height_ = 720;
  int frame_fps_ = 30;
  int timer_period_ms_ = 33;
  int stable_detection_count_ = 5;
  int max_tracking_misses_ = 5;

  double square_size_ = 0.024;
  double axis_length_ = 0.08;
  double detection_scale_ = 0.5;
  double tracking_roi_padding_ratio_ = 0.25;

  bool show_debug_view_ = true;
  bool use_find_chessboard_sb_ = false;
  bool use_tracking_roi_ = true;
  bool flip_image_ = false;
  bool has_last_pose_ = false;
  bool has_last_detection_ = false;

  int stable_hits_ = 0;
  int tracking_misses_ = 0;
};

#endif  // CAMERA_MOUNT_CALIBRATION_NODE_HPP_
