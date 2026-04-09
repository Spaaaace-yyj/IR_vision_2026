#include "camera_mount_calibration/camera_mount_calibration_node.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

using namespace std::chrono_literals;

CameraMountCalibrationNode::CameraMountCalibrationNode()
: Node("camera_mount_calibration_node")
{
  camera_id_ = this->declare_parameter("camera_id", 1);
  frame_width_ = this->declare_parameter("frame_width", 1280);
  frame_height_ = this->declare_parameter("frame_height", 720);
  frame_fps_ = this->declare_parameter("frame_fps", 30);
  timer_period_ms_ = this->declare_parameter("timer_period_ms", 33);
  stable_detection_count_ = this->declare_parameter("stable_detection_count", 5);
  max_tracking_misses_ = this->declare_parameter("max_tracking_misses", 5);

  const int board_cols = this->declare_parameter("board_cols", 7);
  const int board_rows = this->declare_parameter("board_rows", 11);
  square_size_ = this->declare_parameter("square_size", 0.02);
  axis_length_ = this->declare_parameter("axis_length", 0.08);
  detection_scale_ = this->declare_parameter("detection_scale", 0.5);
  tracking_roi_padding_ratio_ =
    this->declare_parameter("tracking_roi_padding_ratio", 0.25);

  show_debug_view_ = this->declare_parameter("show_debug_view", true);
  use_find_chessboard_sb_ = this->declare_parameter("use_find_chessboard_sb", false);
  use_tracking_roi_ = this->declare_parameter("use_tracking_roi", true);
  flip_image_ = this->declare_parameter("flip_image", false);

  board_size_ = cv::Size(board_cols, board_rows);

  const std::vector<double> default_camera_matrix = {
    504.504779, 0.0, 647.473284,
    0.0, 502.266322, 363.157317,
    0.0, 0.0, 1.0
  };
  const std::vector<double> default_dist_coeffs = {
    0.003130, 0.002283, 0.001391, 0.001827, 0.0
  };

  std::vector<double> camera_matrix_values =
    this->declare_parameter<std::vector<double>>("camera_matrix", default_camera_matrix);
  std::vector<double> dist_coeff_values =
    this->declare_parameter<std::vector<double>>("dist_coeffs", default_dist_coeffs);

  if (camera_matrix_values.size() != 9) {
    RCLCPP_WARN(
      this->get_logger(),
      "camera_matrix must contain 9 values, fallback to default intrinsics.");
    camera_matrix_values = default_camera_matrix;
  }

  if (dist_coeff_values.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "dist_coeffs is empty, fallback to default distortion coefficients.");
    dist_coeff_values = default_dist_coeffs;
  }

  camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_values.data()).clone();
  dist_coeffs_ = cv::Mat(1, static_cast<int>(dist_coeff_values.size()), CV_64F, dist_coeff_values.data()).clone();

  if (board_size_.width < 2 || board_size_.height < 2 || square_size_ <= 0.0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid board parameters. board_cols=%d board_rows=%d square_size=%.4f",
      board_size_.width,
      board_size_.height,
      square_size_);
    return;
  }

  detection_scale_ = std::clamp(detection_scale_, 0.2, 1.0);
  tracking_roi_padding_ratio_ = std::clamp(tracking_roi_padding_ratio_, 0.0, 1.0);
  board_object_points_ = createBoardObjectPoints(board_size_, square_size_);

  if (!cap_.open(camera_id_, cv::CAP_V4L2) && !cap_.open(camera_id_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open USB camera %d", camera_id_);
    return;
  }

  cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
  cap_.set(cv::CAP_PROP_FPS, frame_fps_);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_period_ms_),
    std::bind(&CameraMountCalibrationNode::timerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "camera_mount_calibration ready. board=%dx%d square=%.4f m scale=%.2f "
    "tracking_roi=%s. Yaw is relative to the chessboard orientation on the horizontal plane.",
    board_size_.width,
    board_size_.height,
    square_size_,
    detection_scale_,
    use_tracking_roi_ ? "on" : "off");
}

CameraMountCalibrationNode::~CameraMountCalibrationNode()
{
  if (cap_.isOpened()) {
    cap_.release();
  }

  if (show_debug_view_) {
    cv::destroyAllWindows();
  }
}

void CameraMountCalibrationNode::timerCallback()
{
  static int fps = 0;
  static double latency_sum_ms = 0.0;
  static int latency_count = 0;
  static auto start_time = this->now();
  const auto t0 = std::chrono::steady_clock::now();

  if (!cap_.isOpened()) {
    return;
  }

  cv::Mat frame;
  cap_.grab();
  if (!cap_.retrieve(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Failed to read frame from USB camera.");
    return;
  }

  if (flip_image_) {
    cv::flip(frame, frame, -1);
  }

  fps++;

  cv::Mat gray_image;
  cv::cvtColor(frame, gray_image, cv::COLOR_BGR2GRAY);

  std::vector<cv::Point2f> corners;
  const bool board_found = detectChessboard(gray_image, corners);

  cv::Mat display_frame;
  if (show_debug_view_) {
    display_frame = frame.clone();
  }

  if (!board_found) {
    stable_hits_ = 0;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1500,
      "Chessboard not found.");

    if (show_debug_view_) {
      cv::putText(
        display_frame,
        "Chessboard not found",
        cv::Point(20, 40),
        cv::FONT_HERSHEY_SIMPLEX,
        0.9,
        cv::Scalar(0, 0, 255),
        2);
      cv::imshow("CameraMountCalibration", display_frame);
      cv::waitKey(1);
    }
    return;
  }

  cv::Mat rvec = has_last_pose_ ? last_rvec_.clone() : cv::Mat();
  cv::Mat tvec = has_last_pose_ ? last_tvec_.clone() : cv::Mat();
  const bool solved = cv::solvePnP(
    board_object_points_,
    corners,
    camera_matrix_,
    dist_coeffs_,
    rvec,
    tvec,
    has_last_pose_,
    cv::SOLVEPNP_ITERATIVE);

  if (!solved) {
    stable_hits_ = 0;

    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1500,
      "Chessboard detected but solvePnP failed.");

    if (show_debug_view_) {
      cv::drawChessboardCorners(display_frame, board_size_, corners, true);
      cv::imshow("CameraMountCalibration", display_frame);
      cv::waitKey(1);
    }

    has_last_pose_ = false;
    return;
  }

  last_rvec_ = rvec.clone();
  last_tvec_ = tvec.clone();
  has_last_pose_ = true;
  stable_hits_++;

  const cv::Matx33d rotation_camera_from_board = rodriguesToMatrix(rvec);
  const cv::Matx33d rotation_board_from_optical = rotation_camera_from_board.t();

  // Convert OpenCV optical frame (x right, y down, z forward) to a more intuitive
  // camera mount frame (x forward, y left, z up).
  const cv::Matx33d rotation_mount_from_optical = opticalFrameToMountFrameRotation();
  const cv::Matx33d rotation_optical_from_mount = rotation_mount_from_optical.t();
  const cv::Matx33d rotation_board_from_mount =
    rotation_board_from_optical * rotation_optical_from_mount;

  const cv::Vec3d board_origin_in_optical = matToVec3d(tvec);
  const cv::Vec3d camera_position_in_board =
    rotation_board_from_optical * (-board_origin_in_optical);

  const cv::Vec3d mount_rpy_rad = rotationMatrixToRpy(rotation_board_from_mount);

  if (stable_hits_ >= stable_detection_count_) {
    const cv::Vec3d mount_rpy_deg(
      mount_rpy_rad[0] * 180.0 / CV_PI,
      mount_rpy_rad[1] * 180.0 / CV_PI,
      mount_rpy_rad[2] * 180.0 / CV_PI);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Camera mount RPY relative to board [deg]: roll=%.2f pitch=%.2f yaw=%.2f | "
      "camera position in board [m]: x=%.3f y=%.3f z=%.3f",
      mount_rpy_deg[0],
      mount_rpy_deg[1],
      mount_rpy_deg[2],
      camera_position_in_board[0],
      camera_position_in_board[1],
      camera_position_in_board[2]);
  }

  if (show_debug_view_) {
    cv::drawChessboardCorners(display_frame, board_size_, corners, true);
    cv::drawFrameAxes(
      display_frame,
      camera_matrix_,
      dist_coeffs_,
      rvec,
      tvec,
      static_cast<float>(axis_length_));

    const cv::Vec3d mount_rpy_deg(
      mount_rpy_rad[0] * 180.0 / CV_PI,
      mount_rpy_rad[1] * 180.0 / CV_PI,
      mount_rpy_rad[2] * 180.0 / CV_PI);

    auto draw_line = [&display_frame](const std::string & text, int line_index) {
      cv::putText(
        display_frame,
        text,
        cv::Point(20, 35 + line_index * 28),
        cv::FONT_HERSHEY_SIMPLEX,
        0.7,
        cv::Scalar(0, 255, 0),
        2);
    };

    std::ostringstream position_ss;
    position_ss << std::fixed << std::setprecision(3)
                << "pos[m] x=" << camera_position_in_board[0]
                << " y=" << camera_position_in_board[1]
                << " z=" << camera_position_in_board[2];

    std::ostringstream rpy_ss;
    rpy_ss << std::fixed << std::setprecision(2)
           << "rpy[deg] r=" << mount_rpy_deg[0]
           << " p=" << mount_rpy_deg[1]
           << " y=" << mount_rpy_deg[2];

    draw_line(position_ss.str(), 0);
    draw_line(rpy_ss.str(), 1);
    draw_line("stable hits: " + std::to_string(stable_hits_), 2);

    cv::imshow("CameraMountCalibration", display_frame);
    cv::waitKey(1);
  }

  const double latency_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t0).count();
  latency_sum_ms += latency_ms;
  latency_count++;

  const auto now = this->now();
  if (now - start_time >= rclcpp::Duration::from_seconds(1.0)) {
    const double avg_latency = latency_count > 0 ?
      latency_sum_ms / static_cast<double>(latency_count) : 0.0;
    RCLCPP_INFO(
      this->get_logger(),
      "FPS: %d, avg latency: %.2f ms, tracking: %s",
      fps,
      avg_latency,
      has_last_detection_ ? "locked" : "search");
    fps = 0;
    latency_sum_ms = 0.0;
    latency_count = 0;
    start_time = now;
  }
}

bool CameraMountCalibrationNode::detectChessboard(
  const cv::Mat & gray_image,
  std::vector<cv::Point2f> & corners)
{
  corners.clear();

  if (use_tracking_roi_ && has_last_detection_ && last_board_roi_.area() > 0) {
    const cv::Rect search_roi = expandRect(
      last_board_roi_,
      gray_image.size(),
      static_cast<float>(tracking_roi_padding_ratio_));

    std::vector<cv::Point2f> roi_corners;
    if (detectChessboardScaled(gray_image(search_roi), roi_corners)) {
      for (auto & corner : roi_corners) {
        corner.x += static_cast<float>(search_roi.x);
        corner.y += static_cast<float>(search_roi.y);
      }
      refineCorners(gray_image, roi_corners);
      corners = roi_corners;
      updateTrackingState(corners);
      tracking_misses_ = 0;
      return true;
    }
  }

  if (detectChessboardScaled(gray_image, corners)) {
    refineCorners(gray_image, corners);
    updateTrackingState(corners);
    tracking_misses_ = 0;
    return true;
  }

  tracking_misses_++;
  if (tracking_misses_ > max_tracking_misses_) {
    has_last_detection_ = false;
    last_corners_.clear();
    last_board_roi_ = cv::Rect();
  }

  return false;
}

bool CameraMountCalibrationNode::detectChessboardScaled(
  const cv::Mat & gray_image,
  std::vector<cv::Point2f> & corners) const
{
  corners.clear();

  double scale = detection_scale_;
  if (gray_image.cols <= 800 || gray_image.rows <= 600) {
    scale = 1.0;
  }

  cv::Mat work_image;
  if (scale < 0.999) {
    cv::resize(
      gray_image,
      work_image,
      cv::Size(),
      scale,
      scale,
      cv::INTER_AREA);
  } else {
    work_image = gray_image;
    scale = 1.0;
  }

  bool found = false;

  if (use_find_chessboard_sb_) {
    found = cv::findChessboardCornersSB(
      work_image,
      board_size_,
      corners,
      cv::CALIB_CB_NORMALIZE_IMAGE);
  }

  if (!found) {
    found = cv::findChessboardCorners(
      work_image,
      board_size_,
      corners,
      cv::CALIB_CB_ADAPTIVE_THRESH |
      cv::CALIB_CB_NORMALIZE_IMAGE |
      cv::CALIB_CB_FAST_CHECK);
  }

  if (found && scale < 0.999) {
    const float inv_scale = static_cast<float>(1.0 / scale);
    for (auto & corner : corners) {
      corner.x *= inv_scale;
      corner.y *= inv_scale;
    }
  }

  return found;
}

void CameraMountCalibrationNode::refineCorners(
  const cv::Mat & gray_image,
  std::vector<cv::Point2f> & corners) const
{
  if (corners.empty()) {
    return;
  }

  cv::cornerSubPix(
    gray_image,
    corners,
    cv::Size(5, 5),
    cv::Size(-1, -1),
    cv::TermCriteria(
      cv::TermCriteria::EPS + cv::TermCriteria::COUNT,
      20,
      0.05));
}

void CameraMountCalibrationNode::updateTrackingState(const std::vector<cv::Point2f> & corners)
{
  last_corners_ = corners;
  last_board_roi_ = cv::boundingRect(corners);
  has_last_detection_ = true;
}

cv::Rect CameraMountCalibrationNode::expandRect(
  const cv::Rect & rect,
  const cv::Size & image_size,
  float padding_ratio)
{
  if (rect.area() <= 0) {
    return rect;
  }

  const int pad_x = std::max(10, static_cast<int>(std::lround(rect.width * padding_ratio)));
  const int pad_y = std::max(10, static_cast<int>(std::lround(rect.height * padding_ratio)));

  cv::Rect expanded(
    rect.x - pad_x,
    rect.y - pad_y,
    rect.width + 2 * pad_x,
    rect.height + 2 * pad_y);

  expanded &= cv::Rect(0, 0, image_size.width, image_size.height);
  return expanded;
}

std::vector<cv::Point3f> CameraMountCalibrationNode::createBoardObjectPoints(
  const cv::Size & board_size,
  double square_size)
{
  std::vector<cv::Point3f> object_points;
  object_points.reserve(static_cast<size_t>(board_size.width * board_size.height));

  for (int row = 0; row < board_size.height; ++row) {
    for (int col = 0; col < board_size.width; ++col) {
      object_points.emplace_back(
        static_cast<float>(col * square_size),
        static_cast<float>(row * square_size),
        0.0F);
    }
  }

  return object_points;
}

cv::Matx33d CameraMountCalibrationNode::opticalFrameToMountFrameRotation()
{
  return cv::Matx33d(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
}

cv::Matx33d CameraMountCalibrationNode::rodriguesToMatrix(const cv::Mat & rvec)
{
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);

  return cv::Matx33d(
    rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
    rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
    rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));
}

cv::Vec3d CameraMountCalibrationNode::rotationMatrixToRpy(const cv::Matx33d & rotation)
{
  const double sy = std::sqrt(
    rotation(0, 0) * rotation(0, 0) + rotation(1, 0) * rotation(1, 0));
  const bool singular = sy < 1e-6;

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;

  if (!singular) {
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    pitch = std::atan2(-rotation(2, 0), sy);
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    roll = std::atan2(-rotation(1, 2), rotation(1, 1));
    pitch = std::atan2(-rotation(2, 0), sy);
    yaw = 0.0;
  }

  return cv::Vec3d(roll, pitch, yaw);
}

cv::Vec3d CameraMountCalibrationNode::matToVec3d(const cv::Mat & mat)
{
  return cv::Vec3d(
    mat.at<double>(0, 0),
    mat.at<double>(1, 0),
    mat.at<double>(2, 0));
}

std::string CameraMountCalibrationNode::formatRpyDegrees(const cv::Vec3d & rpy_rad)
{
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2)
     << "roll=" << rpy_rad[0] * 180.0 / CV_PI
     << " pitch=" << rpy_rad[1] * 180.0 / CV_PI
     << " yaw=" << rpy_rad[2] * 180.0 / CV_PI;
  return ss.str();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraMountCalibrationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
