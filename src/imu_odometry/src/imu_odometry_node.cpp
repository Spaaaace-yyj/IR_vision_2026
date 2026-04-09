#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

tf2::Quaternion NormalizeQuaternion(const tf2::Quaternion & quaternion)
{
  tf2::Quaternion normalized = quaternion;
  if (normalized.length2() < 1.0e-12) {
    normalized.setValue(0.0, 0.0, 0.0, 1.0);
    return normalized;
  }
  normalized.normalize();
  return normalized;
}

tf2::Quaternion IntegrateQuaternion(
  const tf2::Quaternion & quaternion,
  const tf2::Vector3 & angular_velocity,
  double dt)
{
  const double angle = angular_velocity.length() * dt;
  if (angle < 1.0e-12) {
    return quaternion;
  }

  tf2::Vector3 axis = angular_velocity.normalized();
  tf2::Quaternion delta(axis, angle);
  return NormalizeQuaternion(quaternion * delta);
}

double QuaternionToYaw(const tf2::Quaternion & quaternion)
{
  return tf2::getYaw(quaternion);
}

tf2::Quaternion YawToQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return quaternion;
}

double WrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

tf2::Vector3 RotateVector(const tf2::Quaternion & quaternion, const tf2::Vector3 & vector)
{
  return tf2::quatRotate(quaternion, vector);
}

tf2::Vector3 InverseRotateVector(
  const tf2::Quaternion & quaternion,
  const tf2::Vector3 & vector)
{
  return tf2::quatRotate(quaternion.inverse(), vector);
}

tf2::Vector3 RotateByYaw(double yaw, const tf2::Vector3 & vector)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return tf2::Vector3(
    c * vector.x() - s * vector.y(),
    s * vector.x() + c * vector.y(),
    vector.z());
}

bool IsFiniteQuaternion(const tf2::Quaternion & quaternion)
{
  return std::isfinite(quaternion.x()) &&
         std::isfinite(quaternion.y()) &&
         std::isfinite(quaternion.z()) &&
         std::isfinite(quaternion.w());
}

}  // namespace

class ImuOdometryNode : public rclcpp::Node
{
public:
  ImuOdometryNode()
  : Node("imu_odometry_node"),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    declare_parameter<std::string>("imu_topic", "/imu/data_raw");
    declare_parameter<std::string>("odom_topic", "/imu/odometry");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<bool>("use_imu_orientation", true);
    declare_parameter<double>("gravity_magnitude", 9.80665);
    declare_parameter<double>("max_integration_dt", 0.1);
    declare_parameter<double>("stationary_accel_threshold", 0.35);
    declare_parameter<double>("stationary_gyro_threshold", 0.12);
    declare_parameter<double>("stationary_time_threshold", 0.25);
    declare_parameter<double>("stationary_velocity_threshold", 0.08);
    declare_parameter<double>("stationary_jerk_threshold", 1.2);
    declare_parameter<double>("stationary_filter_alpha", 0.2);
    declare_parameter<double>("stationary_velocity_decay", 12.0);
    declare_parameter<double>("bias_correction_time_constant", 8.0);
    declare_parameter<double>("accel_low_pass_alpha", 0.35);
    declare_parameter<double>("velocity_damping", 0.0);
    declare_parameter<std::string>("reset_service_name", "reset_imu_odometry");

    imu_topic_ = get_parameter("imu_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    use_imu_orientation_ = get_parameter("use_imu_orientation").as_bool();
    gravity_magnitude_ = get_parameter("gravity_magnitude").as_double();
    max_integration_dt_ = get_parameter("max_integration_dt").as_double();
    stationary_accel_threshold_ =
      get_parameter("stationary_accel_threshold").as_double();
    stationary_gyro_threshold_ =
      get_parameter("stationary_gyro_threshold").as_double();
    stationary_time_threshold_ =
      get_parameter("stationary_time_threshold").as_double();
    stationary_velocity_threshold_ =
      get_parameter("stationary_velocity_threshold").as_double();
    stationary_jerk_threshold_ =
      get_parameter("stationary_jerk_threshold").as_double();
    stationary_filter_alpha_ =
      get_parameter("stationary_filter_alpha").as_double();
    stationary_velocity_decay_ =
      get_parameter("stationary_velocity_decay").as_double();
    bias_correction_time_constant_ =
      get_parameter("bias_correction_time_constant").as_double();
    accel_low_pass_alpha_ = get_parameter("accel_low_pass_alpha").as_double();
    velocity_damping_ = get_parameter("velocity_damping").as_double();
    reset_service_name_ = get_parameter("reset_service_name").as_string();

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuOdometryNode::ImuCallback, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      reset_service_name_,
      std::bind(
        &ImuOdometryNode::HandleResetService,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    orientation_.setValue(0.0, 0.0, 0.0, 1.0);
    latest_angular_velocity_.setValue(0.0, 0.0, 0.0);
    gyro_bias_.setValue(0.0, 0.0, 0.0);
    accel_bias_.setValue(0.0, 0.0, 0.0);
    prev_planar_accel_.setValue(0.0, 0.0, 0.0);
    filtered_accel_norm_ = 0.0;
    filtered_gyro_norm_ = 0.0;
    filtered_planar_jerk_ = 0.0;

    RCLCPP_INFO(
      get_logger(),
      "IMU odometry ready: imu_topic=%s odom_topic=%s tf=%s service=%s",
      imu_topic_.c_str(),
      odom_topic_.c_str(),
      publish_tf_ ? "enabled" : "disabled",
      reset_service_name_.c_str());
  }

private:
  void HandleResetService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    ResetState();
    response->success = true;
    response->message = "IMU odometry reset.";
  }

  void ResetState()
  {
    position_x_ = 0.0;
    position_y_ = 0.0;
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    prev_planar_accel_.setValue(0.0, 0.0, 0.0);
    integration_time_ = 0.0;
    stationary_duration_ = 0.0;
    has_last_stamp_ = false;
    reference_yaw_ = latest_yaw_;
    has_reference_yaw_ = true;
    filtered_accel_norm_ = 0.0;
    filtered_gyro_norm_ = 0.0;
    filtered_planar_jerk_ = 0.0;

    RCLCPP_INFO(
      get_logger(),
      "Odometry state reset. reference_yaw=%.3f rad",
      reference_yaw_);
  }

  bool OrientationIsValid(const sensor_msgs::msg::Imu & msg) const
  {
    if (msg.orientation_covariance[0] == -1.0) {
      return false;
    }

    const tf2::Quaternion quaternion(
      msg.orientation.x,
      msg.orientation.y,
      msg.orientation.z,
      msg.orientation.w);
    return IsFiniteQuaternion(quaternion) && quaternion.length2() > 1.0e-12;
  }

  tf2::Quaternion ResolveOrientation(
    const sensor_msgs::msg::Imu & msg,
    const tf2::Vector3 & raw_gyro,
    const rclcpp::Time & stamp)
  {
    if (use_imu_orientation_ && OrientationIsValid(msg)) {
      return NormalizeQuaternion(tf2::Quaternion(
          msg.orientation.x,
          msg.orientation.y,
          msg.orientation.z,
          msg.orientation.w));
    }

    if (!has_last_stamp_) {
      return orientation_;
    }

    const double dt = (stamp - last_stamp_).seconds();
    if (dt <= 0.0 || dt > max_integration_dt_) {
      return orientation_;
    }

    return IntegrateQuaternion(orientation_, raw_gyro - gyro_bias_, dt);
  }

  void UpdateBiasEstimates(
    const tf2::Vector3 & raw_gyro,
    const tf2::Vector3 & raw_accel,
    const tf2::Vector3 & expected_gravity_body,
    double dt)
  {
    const double tau = std::max(1.0e-3, bias_correction_time_constant_);
    const double rate = 1.0 - std::exp(-dt / tau);
    gyro_bias_ = gyro_bias_ * (1.0 - rate) + raw_gyro * rate;
    const tf2::Vector3 accel_bias_target = raw_accel - expected_gravity_body;
    accel_bias_ = accel_bias_ * (1.0 - rate) + accel_bias_target * rate;
  }

  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    rclcpp::Time stamp(msg->header.stamp);
    if (stamp.nanoseconds() == 0) {
      stamp = now();
    }

    const tf2::Vector3 raw_accel(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z);
    const tf2::Vector3 raw_gyro(
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z);

    latest_angular_velocity_ = raw_gyro;

    const double previous_yaw = latest_yaw_;
    const tf2::Quaternion orientation = ResolveOrientation(*msg, raw_gyro, stamp);
    orientation_ = orientation;
    latest_yaw_ = QuaternionToYaw(orientation_);

    if (!has_reference_yaw_) {
      reference_yaw_ = latest_yaw_;
      has_reference_yaw_ = true;
    }

    if (!has_last_stamp_) {
      latest_yaw_rate_ = 0.0;
      last_stamp_ = stamp;
      has_last_stamp_ = true;
      PublishOdometry(stamp, orientation_);
      return;
    }

    const double dt = (stamp - last_stamp_).seconds();
    last_stamp_ = stamp;
    if (dt <= 0.0 || dt > max_integration_dt_) {
      latest_yaw_rate_ = 0.0;
      prev_planar_accel_.setValue(0.0, 0.0, 0.0);
      filtered_accel_norm_ = 0.0;
      filtered_gyro_norm_ = 0.0;
      filtered_planar_jerk_ = 0.0;
      PublishOdometry(stamp, orientation_);
      return;
    }

    integration_time_ += dt;
    latest_yaw_rate_ = WrapAngle(latest_yaw_ - previous_yaw) / dt;

    const tf2::Vector3 corrected_gyro = raw_gyro - gyro_bias_;
    const tf2::Vector3 gravity_world(0.0, 0.0, gravity_magnitude_);
    const tf2::Vector3 expected_gravity_body =
      InverseRotateVector(orientation_, gravity_world);
    const tf2::Vector3 free_accel_body =
      raw_accel - accel_bias_ - expected_gravity_body;
    const tf2::Vector3 world_free_accel =
      RotateVector(orientation_, free_accel_body);
    const tf2::Vector3 odom_accel =
      RotateByYaw(-reference_yaw_, world_free_accel);
    tf2::Vector3 planar_accel(odom_accel.x(), odom_accel.y(), 0.0);
    planar_accel =
      planar_accel * accel_low_pass_alpha_ +
      prev_planar_accel_ * (1.0 - accel_low_pass_alpha_);

    const double planar_jerk =
      (planar_accel - prev_planar_accel_).length() / std::max(dt, 1.0e-6);
    filtered_accel_norm_ =
      filtered_accel_norm_ * (1.0 - stationary_filter_alpha_) +
      free_accel_body.length() * stationary_filter_alpha_;
    filtered_gyro_norm_ =
      filtered_gyro_norm_ * (1.0 - stationary_filter_alpha_) +
      corrected_gyro.length() * stationary_filter_alpha_;
    filtered_planar_jerk_ =
      filtered_planar_jerk_ * (1.0 - stationary_filter_alpha_) +
      planar_jerk * stationary_filter_alpha_;

    const bool imu_quasi_static =
      filtered_gyro_norm_ < stationary_gyro_threshold_ &&
      filtered_accel_norm_ < stationary_accel_threshold_ &&
      filtered_planar_jerk_ < stationary_jerk_threshold_;

    if (imu_quasi_static) {
      stationary_duration_ += dt;
      UpdateBiasEstimates(raw_gyro, raw_accel, expected_gravity_body, dt);
    } else {
      stationary_duration_ = 0.0;
    }

    const bool zero_velocity_update =
      imu_quasi_static && stationary_duration_ >= stationary_time_threshold_;

    if (imu_quasi_static) {
      const double decay =
        std::exp(-std::max(0.0, stationary_velocity_decay_) * dt);
      velocity_x_ *= decay;
      velocity_y_ *= decay;

      if (zero_velocity_update ||
          std::hypot(velocity_x_, velocity_y_) < stationary_velocity_threshold_)
      {
        velocity_x_ = 0.0;
        velocity_y_ = 0.0;
      }

      planar_accel.setValue(0.0, 0.0, 0.0);
    } else {
      const double new_velocity_x =
        (velocity_x_ + 0.5 * (prev_planar_accel_.x() + planar_accel.x()) * dt) *
        std::exp(-velocity_damping_ * dt);
      const double new_velocity_y =
        (velocity_y_ + 0.5 * (prev_planar_accel_.y() + planar_accel.y()) * dt) *
        std::exp(-velocity_damping_ * dt);

      position_x_ += 0.5 * (velocity_x_ + new_velocity_x) * dt;
      position_y_ += 0.5 * (velocity_y_ + new_velocity_y) * dt;
      velocity_x_ = new_velocity_x;
      velocity_y_ = new_velocity_y;
    }

    prev_planar_accel_ = planar_accel;
    PublishOdometry(stamp, orientation_);
  }

  void PublishOdometry(const rclcpp::Time & stamp, const tf2::Quaternion & orientation)
  {
    const double relative_yaw = WrapAngle(QuaternionToYaw(orientation) - reference_yaw_);
    const tf2::Quaternion planar_orientation = YawToQuaternion(relative_yaw);
    const tf2::Vector3 odom_velocity(velocity_x_, velocity_y_, 0.0);
    const tf2::Vector3 body_velocity =
      InverseRotateVector(planar_orientation, odom_velocity);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;
    odom_msg.pose.pose.position.x = position_x_;
    odom_msg.pose.pose.position.y = position_y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf2::toMsg(planar_orientation);
    odom_msg.twist.twist.linear.x = body_velocity.x();
    odom_msg.twist.twist.linear.y = body_velocity.y();
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = latest_yaw_rate_;

    const double position_variance = 0.05 + 0.03 * integration_time_;
    const double yaw_variance = 0.03 + 0.015 * integration_time_;
    const double velocity_variance = 0.05 + 0.04 * integration_time_;
    odom_msg.pose.covariance[0] = position_variance;
    odom_msg.pose.covariance[7] = position_variance;
    odom_msg.pose.covariance[14] = 1.0e-3;
    odom_msg.pose.covariance[21] = 1.0;
    odom_msg.pose.covariance[28] = 1.0;
    odom_msg.pose.covariance[35] = yaw_variance;
    odom_msg.twist.covariance[0] = velocity_variance;
    odom_msg.twist.covariance[7] = velocity_variance;
    odom_msg.twist.covariance[14] = 1.0e-3;
    odom_msg.twist.covariance[21] = 1.0;
    odom_msg.twist.covariance[28] = 1.0;
    odom_msg.twist.covariance[35] = yaw_variance;

    odom_publisher_->publish(odom_msg);

    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = position_x_;
    transform.transform.translation.y = position_y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  std::string imu_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string reset_service_name_;

  bool publish_tf_{true};
  bool use_imu_orientation_{true};
  bool has_last_stamp_{false};
  bool has_reference_yaw_{false};

  double gravity_magnitude_{9.80665};
  double max_integration_dt_{0.1};
  double stationary_accel_threshold_{0.35};
  double stationary_gyro_threshold_{0.12};
  double stationary_time_threshold_{0.25};
  double stationary_velocity_threshold_{0.08};
  double stationary_jerk_threshold_{1.2};
  double stationary_filter_alpha_{0.2};
  double stationary_velocity_decay_{12.0};
  double bias_correction_time_constant_{8.0};
  double accel_low_pass_alpha_{0.35};
  double velocity_damping_{0.0};

  double reference_yaw_{0.0};
  double latest_yaw_{0.0};
  double latest_yaw_rate_{0.0};
  double position_x_{0.0};
  double position_y_{0.0};
  double velocity_x_{0.0};
  double velocity_y_{0.0};
  double stationary_duration_{0.0};
  double integration_time_{0.0};
  double filtered_accel_norm_{0.0};
  double filtered_gyro_norm_{0.0};
  double filtered_planar_jerk_{0.0};

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  tf2::Quaternion orientation_;
  tf2::Vector3 latest_angular_velocity_;
  tf2::Vector3 gyro_bias_;
  tf2::Vector3 accel_bias_;
  tf2::Vector3 prev_planar_accel_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImuOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
