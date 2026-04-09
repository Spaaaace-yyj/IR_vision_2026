#include "../common/hik_camera.h"

// ROS2相关头文件
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// 其他依赖头文件
#include <fmt/format.h>
#include <rcpputils/filesystem_helper.hpp>
#include <csignal>
#include <atomic>

// 添加服务相关的头文件
#include "mvcc_camera_ros2_interface/srv/get_float_value.hpp"
#include "mvcc_camera_ros2_interface/srv/set_float_value.hpp"
#include "mvcc_camera_ros2_interface/srv/get_string_value.hpp"
#include "mvcc_camera_ros2_interface/srv/set_string_value.hpp"
#include "mvcc_camera_ros2_interface/srv/execute_command.hpp"
#include "mvcc_camera_ros2_interface/srv/get_enum_value.hpp"
#include "mvcc_camera_ros2_interface/srv/set_enum_value.hpp"
#include "mvcc_camera_ros2_interface/srv/get_integer_value.hpp"
#include "mvcc_camera_ros2_interface/srv/set_integer_value.hpp"
#include "mvcc_camera_ros2_interface/srv/get_bool_value.hpp"
#include "mvcc_camera_ros2_interface/srv/set_bool_value.hpp"

// 添加消息相关的头文件
#include "mvcc_camera_ros2_interface/msg/frame_info.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

// 相机chuankID
#define CAMERA_CHUNK_ID_WIDTH               0xa5a5010a
#define CAMERA_CHUNK_ID_HEIGHT              0xa5a5010b
#define CAMERA_CHUNK_ID_OFFSET_X            0xa5a5010c
#define CAMERA_CHUNK_ID_OFFSET_Y            0xa5a5010d


// 全局信号标志
std::atomic<bool> g_shutdown_requested(false);

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("mvcc_camera_image_subscriber"), is_shutting_down_(false)
    {
        // 注册信号处理器
        setupSignalHandlers();

        // 获取工作空间路径，构建保存目录
        std::string home = std::getenv("HOME") ? std::getenv("HOME") : ".";
        str_save_path_ = home + "/mvcc_camera_images";
               
        if (rcpputils::fs::create_directories(rcpputils::fs::path(str_save_path_))) 
        {
            RCLCPP_INFO(this->get_logger(), "[Subscriber] Created save directory: %s", str_save_path_.c_str());
        } 
        else
        {
            RCLCPP_WARN(this->get_logger(), "[Subscriber] Directory already exists or cannot create: %s", str_save_path_.c_str());
        }

        // 1.  初始化  订阅者，指定队列长度为100，与frame_info_sub_保持一致
        image_sub_ = this->create_subscription<ImageMsg>(
            "/mvcc_camera/Image", 100, 
            std::bind(&ImageSubscriber::imageCallback, this, std::placeholders::_1));

        info_sub_ = this->create_subscription<InfoMsg>(
            "/mvcc_camera/ImageInfo", 100, 
            std::bind(&ImageSubscriber::infoCallback, this, std::placeholders::_1));

        // 附加: 其他订阅方法：同步订阅 image_sub_ 和 info_sub_ ; 配置合适的同步策略，否则容易导致队列阻塞

        
        // 2. 创建FrameInfo订阅者 (私有订阅，附加包含更多图像信息)
        frame_info_sub_ = this->create_subscription<mvcc_camera_ros2_interface::msg::FrameInfo>(
                            "/mvcc_camera/FrameInfo", 100, 
                            std::bind(&ImageSubscriber::frameInfoCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[Subscriber] Waiting for synchronized Image and CameraInfo...");

        // 3. 创建参数配置/读取客户端（可选功能）
        get_float_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::GetFloatValue>("/get_float_value");
        set_float_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::SetFloatValue>("/set_float_value");
        get_string_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::GetStringValue>("/get_string_value");
        set_string_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::SetStringValue>("/set_string_value");
        execute_command_client_ = this->create_client<mvcc_camera_ros2_interface::srv::ExecuteCommand>("/execute_command");
        get_enum_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::GetEnumValue>("/get_enum_value");
        set_enum_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::SetEnumValue>("/set_enum_value");
        get_integer_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::GetIntegerValue>("/get_integer_value");
        set_integer_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::SetIntegerValue>("/set_integer_value");
        get_bool_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::GetBoolValue>("/get_bool_value");
        set_bool_value_client_ = this->create_client<mvcc_camera_ros2_interface::srv::SetBoolValue>("/set_bool_value");
        
        // 启动一个定时器来检查服务可用性，避免阻塞主线程
        service_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() { checkServicesAndInit(); });
        
        RCLCPP_INFO(this->get_logger(), "Image subscriber initialized. Services will be checked in background.");

    }

    ~ImageSubscriber()
    {
        shutdown();
    }

private:
    uint32_t seq_{0};  
    string str_save_path_;
    std::atomic<bool> is_shutting_down_;

    // 客户端
    rclcpp::Client<mvcc_camera_ros2_interface::srv::GetFloatValue>::SharedPtr get_float_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::SetFloatValue>::SharedPtr set_float_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::GetStringValue>::SharedPtr get_string_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::SetStringValue>::SharedPtr set_string_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::ExecuteCommand>::SharedPtr execute_command_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::GetEnumValue>::SharedPtr get_enum_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::SetEnumValue>::SharedPtr set_enum_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::GetIntegerValue>::SharedPtr get_integer_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::SetIntegerValue>::SharedPtr set_integer_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::GetBoolValue>::SharedPtr get_bool_value_client_;
    rclcpp::Client<mvcc_camera_ros2_interface::srv::SetBoolValue>::SharedPtr set_bool_value_client_;

    using ImageMsg = sensor_msgs::msg::Image;
    using InfoMsg = sensor_msgs::msg::CameraInfo;

    // 使用create_subscription方式订阅，指定队列长度为100
    rclcpp::Subscription<ImageMsg>::SharedPtr image_sub_;
    rclcpp::Subscription<InfoMsg>::SharedPtr info_sub_;

    // FrameInfo订阅者
    rclcpp::Subscription<mvcc_camera_ros2_interface::msg::FrameInfo>::SharedPtr frame_info_sub_;

    // 定时器用于检查服务可用性
    rclcpp::TimerBase::SharedPtr service_check_timer_;
    std::atomic<bool> services_checked_{false};

    // 信号处理函数
    static void signalHandler(int signum)
    {
        RCLCPP_INFO(rclcpp::get_logger("signal_handler"), "Received signal %d, shutting down gracefully", signum);
        g_shutdown_requested = true;
        rclcpp::shutdown();
    }

    void setupSignalHandlers()
    {
        signal(SIGINT, signalHandler);
        signal(SIGTERM, signalHandler);
    }

    void shutdown()
    {
        if (is_shutting_down_.exchange(true)) {
            return; // 已经在关闭过程中
        }

        RCLCPP_INFO(this->get_logger(), "Shutting down ImageSubscriber...");


        
        // 重置所有订阅器和客户端
        if (image_sub_) {
            image_sub_.reset();
        }
        
        if (info_sub_) {
            info_sub_.reset();
        }
        
        if (frame_info_sub_) {
            frame_info_sub_.reset();
        }

        if (service_check_timer_) {
            service_check_timer_->cancel();
            service_check_timer_.reset();
        }

        if (get_float_value_client_) {
            get_float_value_client_.reset();
        }

        if (set_float_value_client_) {
            set_float_value_client_.reset();
        }

        if (get_string_value_client_) {
            get_string_value_client_.reset();
        }

        if (set_string_value_client_) {
            set_string_value_client_.reset();
        }

        if (execute_command_client_) {
            execute_command_client_.reset();
        }
        if (get_enum_value_client_) {
            get_enum_value_client_.reset();
        }
        if (set_enum_value_client_) {
            set_enum_value_client_.reset();
        }
        if (get_integer_value_client_) {
            get_integer_value_client_.reset();
        }
        if (set_integer_value_client_) {
            set_integer_value_client_.reset();
        }
        if (get_bool_value_client_) {
            get_bool_value_client_.reset();
        }
        if (set_bool_value_client_) {
            set_bool_value_client_.reset();
        }


        RCLCPP_INFO(this->get_logger(), "ImageSubscriber shutdown complete");
    }

    void imageCallback(const ImageMsg::ConstSharedPtr& image_msg)
    {
        static uint32_t image_seq = 0;
        RCLCPP_INFO(this->get_logger(), "[Subscriber] Get image frameid[%s]  timestamp: %f, seq [%d] ",
                    image_msg->header.frame_id.c_str() , image_msg->header.stamp.sec + image_msg->header.stamp.nanosec * 1e-9, image_seq);


       
        //方式1，保存图像到本地磁盘
        if(image_seq < 3) // seq_ 小于3 图像保存，避免磁盘占用过多磁盘
        {
            
            // 转换为 OpenCV 图像
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                // 自动检测图像编码格式，支持MONO8和RGB8
                cv_ptr = cv_bridge::toCvCopy(image_msg);
            }
            catch (const cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[Subscriber] cv_bridge exception: %s", e.what());
                return;
            }


            // 保存图像
            std::string filename;
            if (cv_ptr->image.channels() == 1) 
            {
                // 灰度图像
                filename = (rcpputils::fs::path(str_save_path_) / 
                           fmt::format("[{}]_Image_mono.png", image_seq)).string();
            } else if (cv_ptr->image.channels() == 3) {
                // RGB图像
                filename = (rcpputils::fs::path(str_save_path_) / 
                           fmt::format("[{}]_Image_rgb.png", image_seq)).string();
            } else {
                // 其他格式
                filename = (rcpputils::fs::path(str_save_path_) / 
                           fmt::format("[{}]_Image.png", image_seq)).string();
            }

            try
            {
                if (cv::imwrite(filename, cv_ptr->image))
                {
                    RCLCPP_INFO(this->get_logger(), "[Subscriber] Saved image: %s", filename.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "[Subscriber] Failed to save image (OpenCV write failed): %s", filename.c_str());
                }
            }
            catch (const cv::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[Subscriber] OpenCV write exception: %s", e.what());
            }            
        }

        #if 0 
        //方式2，保存图像到本地磁盘  （裸数据直接保存)
        if(image_seq < 3)
        {
            // 获取时间戳作为文件名
            auto timestamp = image_msg->header.stamp;
            std::string filename = "image_" + std::to_string(timestamp.sec) + 
                                "_" + std::to_string(timestamp.nanosec) + ".bin";
            
            // 直接保存二进制数据
            std::ofstream file(filename, std::ios::binary);
            file.write(reinterpret_cast<const char*>(image_msg->data.data()), image_msg->data.size());
            file.close();
        }
        #endif 


        image_seq++;
    }

    void infoCallback(const InfoMsg::ConstSharedPtr& info_msg)
    {
        static uint32_t info_seq = 0;
        RCLCPP_INFO(this->get_logger(), "[Subscriber] Got CameraInfo frameid[%s]  timestamp: %f, seq [%d] ",
                    info_msg->header.frame_id.c_str() , info_msg->header.stamp.sec + info_msg->header.stamp.nanosec * 1e-9, info_seq);
        
        info_seq++;
    }

    void frameInfoCallback(const mvcc_camera_ros2_interface::msg::FrameInfo::SharedPtr frame_info_msg)
    {
        static uint32_t frame_info_seq = 0;
        
        RCLCPP_INFO(this->get_logger(), "[frameInfoCallback] Got FrameInfo frame [%d]", frame_info_seq);
        RCLCPP_INFO(this->get_logger(), "  Frame Number: %u", frame_info_msg->n_frame_num);
        RCLCPP_INFO(this->get_logger(), "  Frame Size: %ux%u", frame_info_msg->n_extend_width, frame_info_msg->n_extend_height);
        RCLCPP_INFO(this->get_logger(), "  Frame Length: %lu bytes", frame_info_msg->n_frame_len_ex);
        RCLCPP_INFO(this->get_logger(), "  Pixel Type: %u", frame_info_msg->en_pixel_type);
        RCLCPP_INFO(this->get_logger(), "  Device Timestamp: 0x%08x%08x", 
                   frame_info_msg->n_dev_timestamp_high, frame_info_msg->n_dev_timestamp_low);
        RCLCPP_INFO(this->get_logger(), "  Host Timestamp: %ld", frame_info_msg->n_host_timestamp);
        RCLCPP_INFO(this->get_logger(), "  Exposure Time: %f ms", frame_info_msg->f_exposure_time);
        RCLCPP_INFO(this->get_logger(), "  Gain: %f", frame_info_msg->f_gain);
        RCLCPP_INFO(this->get_logger(), "  Lost Packets: %u", frame_info_msg->n_lost_packet);
        RCLCPP_INFO(this->get_logger(), "  Unparsed Chunks: %u", frame_info_msg->n_unparsed_chunk_num);
        
        // 打印Chunk数据信息
        for (size_t i = 0; i < frame_info_msg->unparsed_chunk_content.size(); ++i) 
        {
            const auto& chunk = frame_info_msg->unparsed_chunk_content[i];
            RCLCPP_INFO(this->get_logger(), "  Chunk[%zu]: ID=%u, Len=%u, DataSize=%zu", 
                       i, chunk.n_chunk_id, chunk.n_chunk_len, chunk.p_chunk_data.size());
                       
            if(4 == chunk.p_chunk_data.size() &&
             (CAMERA_CHUNK_ID_WIDTH == chunk.n_chunk_id || CAMERA_CHUNK_ID_HEIGHT == chunk.n_chunk_id 
                || CAMERA_CHUNK_ID_OFFSET_X == chunk.n_chunk_id || CAMERA_CHUNK_ID_OFFSET_Y == chunk.n_chunk_id))
            {
                // 此处尝试 将 chunk.p_chunk_data 转换为 int 并打印
                // 请根据根据实际chunkid定义进行解析
                int32_t nvalue = 0;
                memcpy(&nvalue, chunk.p_chunk_data.data(), sizeof(int32_t));
                RCLCPP_INFO(this->get_logger(), "  Chunk[%zu]: Data=%d", i, nvalue);
            }       
            
        }

        // 参考 ImageInfoCallback  把图像保存到本地
        if(frame_info_seq < 3)
        {
            // 转换为 OpenCV 图像
            cv_bridge::CvImagePtr cv_ptr;
            try
            {
                // 自动检测图像编码格式，支持MONO8和RGB8
                cv_ptr = cv_bridge::toCvCopy(frame_info_msg->image);
            }
            catch (const cv_bridge::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[frameInfoCallback] cv_bridge exception: %s", e.what());
                return;
            }

            // 保存图像
            std::string filename;
            if (cv_ptr->image.channels() == 1) 
            {
                // 灰度图像
                filename = (rcpputils::fs::path(str_save_path_) / 
                           fmt::format("[{}]_FrameInfo_mono.png", frame_info_seq)).string();
            } else if (cv_ptr->image.channels() == 3) {
                // RGB图像
                filename = (rcpputils::fs::path(str_save_path_) / 
                           fmt::format("[{}]_FrameInfo_rgb.png", frame_info_seq)).string();
            } else {
                // 其他格式
                filename = (rcpputils::fs::path(str_save_path_) / 
                           fmt::format("[{}]_FrameInfo.png", frame_info_seq)).string();
            }

            try
            {
                if (cv::imwrite(filename, cv_ptr->image))
                {
                    RCLCPP_INFO(this->get_logger(), "[frameInfoCallback] Saved FrameInfo image: %s", filename.c_str());
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "[frameInfoCallback] Failed to save FrameInfo image (OpenCV write failed): %s", filename.c_str());
                }
            }
            catch (const cv::Exception& e)
            {
                RCLCPP_ERROR(this->get_logger(), "[frameInfoCallback] OpenCV write exception: %s", e.what());
            }            
        }

        
        frame_info_seq++;
    }


    
    void get_current_param()
    {
        RCLCPP_INFO(this->get_logger(), "******** get_current_param begin ********");
        
        // 逐个独立读取参数，每个参数的读取互不影响
        
        // 1. 读取 float 类型，节点名 "ExposureTime" 的值
        auto get_exposure_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetFloatValue::Request>();
        get_exposure_request->str_key = "ExposureTime";
            
        auto get_exposure_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetFloatValue>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Current ExposureTime: %f", response->value);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get ExposureTime: %s", response->message.c_str());
            }
        };
        get_float_value_client_->async_send_request(get_exposure_request, get_exposure_callback);
        
        // 2. 读取 string 类型，节点名 "DeviceUserID" 的值
        auto get_device_id_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetStringValue::Request>();
        get_device_id_request->str_key = "DeviceUserID";
                
        auto get_device_id_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetStringValue>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "DeviceUserID value: %s", response->value.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get DeviceUserID: %s", response->message.c_str());
            }
        };
        get_string_value_client_->async_send_request(get_device_id_request, get_device_id_callback);
        
        // 3. 读取 int 类型，节点名 "AcquisitionLineRate" 的值
        auto get_line_rate_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetIntegerValue::Request>();
        get_line_rate_request->str_key = "AcquisitionLineRate";
                        
        auto get_line_rate_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetIntegerValue>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "AcquisitionLineRate value: %ld", response->value);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get AcquisitionLineRate: %s", response->message.c_str());
            }
        };
        get_integer_value_client_->async_send_request(get_line_rate_request, get_line_rate_callback);
        
        // 4. 读取 enum 类型，节点名 "UserSetSelector" 的值
        auto get_user_set_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetEnumValue::Request>();
        get_user_set_request->str_key = "UserSetSelector";
                                
        auto get_user_set_callback = [this, get_user_set_request](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetEnumValue>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Get  [%s] value: [%ld], string value: [%s]", 
                get_user_set_request->str_key.c_str(),  response->value, response->str_value.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get  [%s] value, error[%s]",
                 get_user_set_request->str_key.c_str(), response->message.c_str());
            }
        };
        get_enum_value_client_->async_send_request(get_user_set_request, get_user_set_callback);

        // 5. 读取 bool 类型，节点名 "ReverseX" 的值
        auto get_trigger_mode_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetBoolValue::Request>();
        get_trigger_mode_request->str_key = "ReverseX";
                                
        auto get_trigger_mode_callback = [this, get_trigger_mode_request](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetBoolValue>::SharedFuture future) {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Get  [%s] value: [%s]", 
                get_trigger_mode_request->str_key.c_str(),  response->value ? "true" : "false");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get  [%s] value, error[%s]",
                 get_trigger_mode_request->str_key.c_str(), response->message.c_str());
            }
        };
        get_bool_value_client_->async_send_request(get_trigger_mode_request, get_trigger_mode_callback);

        RCLCPP_INFO(this->get_logger(), "******** get_current_param end ********");
    }

    void checkServicesAndInit()
    {
        // 如果已经检查过服务，则不再重复检查
        if (services_checked_.exchange(true)) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Checking service availability...");
        
        // 检查关键服务是否可用
        std::vector<std::string> service_names = {
            "/get_float_value",
            "/set_float_value", 
            "/get_string_value",
            "/set_string_value",
            "/execute_command",
            "/get_enum_value",
            "/set_enum_value",
            "/get_integer_value",
            "/set_integer_value",
            "/get_bool_value",
            "/set_bool_value"
        };

        std::vector<rclcpp::ClientBase::SharedPtr> clients = {
            get_float_value_client_,
            set_float_value_client_,
            get_string_value_client_,
            set_string_value_client_,
            execute_command_client_,
            get_enum_value_client_,
            set_enum_value_client_,
            get_integer_value_client_,
            set_integer_value_client_,
            get_bool_value_client_,
            set_bool_value_client_
        };

        bool all_services_ready = true;
        for (size_t i = 0; i < clients.size(); ++i) {
            if (!clients[i]->wait_for_service(100ms)) {
                all_services_ready = false;
                RCLCPP_WARN(this->get_logger(), "Service not available: %s", service_names[i].c_str());
            }
        }

        if (all_services_ready) {
            RCLCPP_INFO(this->get_logger(), "All services are ready! Getting current parameters...");
            get_current_param();
            // 停止定时器，因为服务已经可用
            if (service_check_timer_) {
                service_check_timer_->cancel();
            }
        } else {
            // 重置标志，允许下次继续检查
            services_checked_.exchange(false);
            RCLCPP_INFO(this->get_logger(), "Some services not available, will check again in 2 seconds...");
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
