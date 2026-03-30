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

    detector_params_->adaptiveThreshWinSizeMin = 3;
    detector_params_->adaptiveThreshWinSizeMax = 23;

    detector_params_->minMarkerPerimeterRate = 0.02;
    detector_params_->maxMarkerPerimeterRate = 4.0;

    detector_params_->polygonalApproxAccuracyRate = 0.03;

    show_thread_ = std::thread(&CarControlNode::debugThread, this);

    RCLCPP_INFO(this->get_logger(), "Success Init Car Control node!");
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

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void CarControlNode::imageCallback()
{
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Empty frame");
        return;
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(frame, detector_dict_, marker_corners, marker_ids, detector_params_);
    
    if(!marker_ids.empty()){
        cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
    }
    
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        latest_frame_ = frame.clone();
    }

    // publishImage(frame);
}

// void CarControlNode::publishImage(const cv::Mat &image)
// {
//     auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
//     msg->header.stamp = this->now();
//     image_publisher_->publish(*msg);
// }

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
