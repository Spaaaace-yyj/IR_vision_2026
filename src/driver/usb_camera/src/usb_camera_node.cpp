#include "../include/usb_camera/uab_camera_node.hpp"

USBCameraNode::USBCameraNode() : Node("usb_camera_node")
{

  cap_.open(1);
  if(!cap_.isOpened()){
    RCLCPP_ERROR(this->get_logger(), "Cannot open camera");
    return; 
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  cap_.set(cv::CAP_PROP_FPS, 30);

  image01_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image01_raw", rclcpp::SensorDataQoS());
  image_timer_ = this->create_wall_timer(33ms, std::bind(&USBCameraNode::USBCameraCallback, this));
  RCLCPP_INFO(this->get_logger(), "Success Init USB Camera node!");
}

void USBCameraNode::USBCameraCallback(){
  
  cv::Mat frame;
  cap_.grab();
  cap_.retrieve(frame);

  if (frame.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Empty frame");
    return;
  }

  cv::imshow("image01", frame);
  cv::waitKey(1);

  // 转 ROS2 Image
  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
  msg->header.stamp = this->now();
  image01_publisher_->publish(*msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<USBCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


