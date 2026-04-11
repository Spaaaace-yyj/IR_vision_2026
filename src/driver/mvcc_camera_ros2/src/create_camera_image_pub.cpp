#include "../common/hik_camera.h"
#include "../common/camera_parameter_service_manager.h"

// ROS2相关头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

// 自定义消息头文件
#include "mvcc_camera_ros2_interface/msg/frame_info.hpp"

#include <algorithm>
#include <sstream>
#include <vector>

using namespace std;

namespace {

bool shouldSkipStartupParameter(const std::string &name)
{
    return name == "cam_sn" ||
           name == "use_sim_time" ||
           name.rfind("qos_overrides.", 0) == 0;
}

std::string summarizeParameterNames(
    const std::vector<std::string> &names,
    std::size_t max_names = 5)
{
    if (names.empty()) {
        return "";
    }

    std::ostringstream summary;
    const auto count = std::min(names.size(), max_names);
    for (std::size_t i = 0; i < count; ++i) {
        if (i != 0) {
            summary << ", ";
        }
        summary << names[i];
    }

    if (names.size() > count) {
        summary << " ...";
    }

    return summary.str();
}

}  // namespace

// 前向声明
class HikCamera;
class CameraParameterServiceManager;

class ImagePublish : public rclcpp::Node
{

public:
    ImagePublish() : Node("mvcc_camera_image_publisher", rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)), camera_(), serial_number_("None"),seq_(0)
    {
        
        RCLCPP_INFO(this->get_logger(), "--------------------- Camera Node Init Begin -----------------------.");

        // 直接获取相机序列号参数，无需预先声明
        // 由于使用了automatically_declare_parameters_from_overrides(true)，参数会自动声明
        // 我们只需要尝试获取参数值，如果获取不到则使用默认值
        if (!this->get_parameter("cam_sn", serial_number_)) {
            serial_number_ = "None";  // 使用默认值
        }
        
        RCLCPP_INFO(this->get_logger(), "Final camera SN: [%s]", serial_number_.c_str());

        
        int nRet = MV_OK;

        if(serial_number_ == "None")
        {
            RCLCPP_INFO(this->get_logger(), "input cam_sn is null, open first camera.");          

            // 默认连接第一个相机
            nRet = camera_.open();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "---------------- open by serial number: [%s] ----------------", serial_number_.c_str());

            // 根据序列号进行连接
            nRet = camera_.open(serial_number_.c_str());
        }

        if(nRet != MV_OK)
        {
            RCLCPP_INFO(this->get_logger(), "open camera failed, [%#x].",nRet);
            camera_.close();
            throw std::runtime_error("Camera connection failed"); 
        }

        

        // 遍历配置所有参数，并输出简洁的启动摘要
        std::vector<std::string> param_names = this->list_parameters({}, 10).names;
        std::size_t applied_param_count = 0;
        std::size_t skipped_param_count = 0;
        std::vector<std::string> failed_param_names;
        for (const auto& name : param_names)
        {
            if (shouldSkipStartupParameter(name))
            {
                ++skipped_param_count;
                continue;
            }

            // 获取参数描述以确定参数类型
            try {
                const rclcpp::Parameter& param = this->get_parameter(name);
                const std::string param_value_str = param.value_to_string();
                const int set_ret = camera_.setCameraParameterbyString(name, param_value_str);
                if (set_ret == MV_OK) {
                    ++applied_param_count;
                } else {
                    failed_param_names.push_back(name);
                }
            }
            catch (const std::exception& e)
            {
                failed_param_names.push_back(name);
                RCLCPP_WARN(this->get_logger(), "Failed to process parameter [%s]: %s", name.c_str(), e.what());
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Camera parameter sync finished: %zu applied, %zu skipped, %zu not applied",
            applied_param_count,
            skipped_param_count,
            failed_param_names.size());
        if (!failed_param_names.empty()) {
            RCLCPP_WARN(
                this->get_logger(),
                "Camera parameters not applied: %s",
                summarizeParameterNames(failed_param_names).c_str());
        }
        
        // 方式1: 注册经过处理后的图像  (接口上报的相机图像已经经过解码和格式转换)
        nRet = camera_.registerProcessedImageCallback(ImagePublish::onNewProcessedFrameReport, this);
        if (nRet != MV_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to register camera callback, [%#x].", nRet);
            camera_.close();
            throw std::runtime_error("Camera callback registration failed");
        }

        // 方式2： 注册相机输出的图像，（图像没有经过处理， 应用层根据需要进行处理后发布)
       // camera_.registerImageCallback(ImagePublish::onNewFrameReport, this);


        RCLCPP_INFO(this->get_logger(), "------------------------ ImagePublish begin ------------------------.");
        
        // 创建发布者[标准]
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/mvcc_camera/Image", 200);
        info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/mvcc_camera/ImageInfo", 200);
        
        // 创建FrameInfo发布者
        frame_info_pub_ = this->create_publisher<mvcc_camera_ros2_interface::msg::FrameInfo>("/mvcc_camera/FrameInfo", 200);
           
        // 创建相机参数服务管理器
        try {
            parameter_service_manager_ = std::make_unique<CameraParameterServiceManager>(this, &camera_);
            RCLCPP_INFO(this->get_logger(), "------------------------ Parameter Services created ------------------------.");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create parameter service manager: %s", e.what());
            camera_.close();
            throw;
        }

        // 开始取流  [默认开启]
        nRet = camera_.start();
        if(nRet != MV_OK)
        {
            RCLCPP_INFO(this->get_logger(), "start camera failed, [%#x].",nRet);
            camera_.close();
            throw std::runtime_error("Camera start failed"); 
        }
    }

     ~ImagePublish()
    {
        // 在析构函数中添加一些延迟，确保所有回调都已完成
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        camera_.stop();
        camera_.close();
        RCLCPP_INFO(this->get_logger(), "[Publisher] Camera stopped and closed.");
    }


public:

    // 注册处理过的图像
    static void onNewProcessedFrameReport(unsigned char * pData, MV_FRAME_OUT_INFO_EX* frame_info, void* pUser)
    {
        if (pUser)
        {
            ImagePublish* publisher = static_cast<ImagePublish*>(pUser);
            publisher->onNewProcessedFrameReport(pData, frame_info);
        }
    }

    //注册相机直接输出图像
    static void onNewFrameReport(unsigned char * pData, MV_FRAME_OUT_INFO_EX* frame_info, void* pUser)
    {
        if (pUser)
        {
            ImagePublish* publisher = static_cast<ImagePublish*>(pUser);
            publisher->onNewFrameProc(pData, frame_info);
        }
    }

private:


    void onNewFrameProc(unsigned char * pData, MV_FRAME_OUT_INFO_EX* frame_info)
    {

        int nRet = MV_OK; 

        // 对图像进行解码处理
        MVCC_ROS2_IMAGE_DECODE_INOUT decode_info;
        memset(&decode_info, 0, sizeof(MVCC_ROS2_IMAGE_DECODE_INOUT));

        decode_info.nWidth = frame_info->nExtendWidth;
        decode_info.nHeight = frame_info->nExtendHeight;
        decode_info.enPixelType = frame_info->enPixelType;
        decode_info.pBufAddr = pData;
        decode_info.nBufLen = frame_info->nFrameLenEx;
        nRet = camera_.imageDecodingProcess(&decode_info);
        if(nRet != MV_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "[onNewFrameProc] imageDecodingProcess failed, [%#x].", nRet);
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "[onNewFrameProc] imageDecodingProcess success.");


#if 0

        // 方式1: 发送相机的原始数据 
        auto message = std::make_unique<sensor_msgs::msg::Image>();
        
        // 设置消息头
        message->header.stamp = this->now();
        message->header.frame_id = "camera_frame";
        
        // 直接填充原始数据
        message->height = decode_info.nHeight;
        message->width = decode_info.nWidth;
        message->encoding = "mono8";  // 或其他合适的编码格式 (根据实际情况调整)
   
        // 关键：直接复制原始数据
        size_t data_size = decode_info.nDestBufLen;
        message->data.resize(data_size);
        memcpy(message->data.data(),  decode_info.pDestBufAddr, data_size);
        
        image_pub_->publish(std::move(message));
#endif 

        // 方式2：发送转换后的图像格式 (需要对图像进行格式转换处理) 
        MVCC_ROS2_IMAGE_CONVERT_INOUT convert_info ;
        memset(&convert_info, 0, sizeof(MVCC_ROS2_IMAGE_CONVERT_INOUT));
        convert_info.nWidth = decode_info.nWidth;  // 使用解码后的尺寸
        convert_info.nHeight = decode_info.nHeight; // 使用解码后的尺寸
        convert_info.enPixelType = decode_info.enDestPixelType; // 使用解码后的像素格式
        convert_info.pBufAddr = decode_info.pDestBufAddr; // 使用解码后的数据地址
        convert_info.nBufLen = decode_info.nDestBufLen; // 使用解码后的数据长度

        if(camera_.IsMonoPixelFormat(decode_info.enDestPixelType))
        {
            convert_info.enDestPixelType = PixelType_Gvsp_Mono8; // 目标像素格式
        }
        else
        {
            convert_info.enDestPixelType = PixelType_Gvsp_RGB8_Packed; // 目标像素格式
        }

        nRet = camera_.imageConvertProcess(&convert_info);
        if(nRet != MV_OK)
        {
            RCLCPP_ERROR(this->get_logger(), "[onNewFrameProc] imageConvertProcess failed, [%#x].", nRet);
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "[onNewFrameProc] imageConvertProcess success.");


        // 发送图像数据到ROS2
        sensor_msgs::msg::Image::SharedPtr image_ptr;
        auto info_ptr = std::make_shared<sensor_msgs::msg::CameraInfo>();
        
        // 设置 Header
        std_msgs::msg::Header header;
        header.stamp = this->now();  // 或从相机获取精确时间戳  
        
        // 根据像素格式创建图像消息
        if(PixelType_Gvsp_Mono8== convert_info.enDestPixelType)
        {
            header.frame_id =  "camera_link";           //普通单目灰度
            // 创建 Mat（引用数据）
            cv::Mat stImgTmp = cv::Mat(cv::Size(convert_info.nWidth, 
                                        convert_info.nHeight), 
                                    CV_8UC1, 
                                    convert_info.pBufAddr);

            // 转为 ROS 2 消息（直接使用智能指针，避免拷贝）
            image_ptr = cv_bridge::CvImage(header, "mono8", stImgTmp).toImageMsg();


        }
        else if(PixelType_Gvsp_RGB8_Packed == convert_info.enDestPixelType)
        {
            header.frame_id = "camera_color_frame";           //普通 彩色相机
            // 创建 Mat（引用数据）
            cv::Mat stImgTmp = cv::Mat(cv::Size(convert_info.nWidth,
                                    convert_info.nHeight), 
                                    CV_8UC3, 
                                    convert_info.pBufAddr);
            // 转为 ROS 2 消息（直接使用智能指针，避免拷贝）
            image_ptr = cv_bridge::CvImage(header, "rgb8", stImgTmp).toImageMsg();
        }
        else
        {
            // 不支持的像素格式
            RCLCPP_ERROR(
                this->get_logger(),
                "Not support pixeltype  [%#x] .",
                static_cast<unsigned int>(convert_info.enDestPixelType));
            return;
        }
        


        // === 2. 创建 CameraInfo 消息 ===
        info_ptr->header = header;
        info_ptr->width = convert_info.nWidth;
        info_ptr->height = convert_info.nHeight;

        // === 发布标准 消息 ===
        image_pub_->publish(*image_ptr);
        info_pub_->publish(*info_ptr);

        // 创建并发布FrameInfo消息
        if(frame_info_pub_ && frame_info_pub_->get_subscription_count() > 0)
        {
            auto frame_info_msg = std::make_shared<mvcc_camera_ros2_interface::msg::FrameInfo>();
            
            // 填充FrameInfo消息
            frame_info_msg->image = *image_ptr;
            frame_info_msg->header = header;
            
            frame_info_msg->n_extend_width = convert_info.nWidth;
            frame_info_msg->n_extend_height = convert_info.nHeight;
            frame_info_msg->n_frame_len_ex = convert_info.nDestBufLen;
            frame_info_msg->en_pixel_type = static_cast<uint32_t>(convert_info.enDestPixelType);


            frame_info_msg->n_frame_num = frame_info->nFrameNum;
            frame_info_msg->n_dev_timestamp_high = frame_info->nDevTimeStampHigh;
            frame_info_msg->n_dev_timestamp_low = frame_info->nDevTimeStampLow;
            frame_info_msg->n_host_timestamp = frame_info->nHostTimeStamp;
            frame_info_msg->n_second_count = frame_info->nSecondCount;
            frame_info_msg->n_cycle_count = frame_info->nCycleCount;
            frame_info_msg->n_cycle_offset = frame_info->nCycleOffset;
            frame_info_msg->f_gain = frame_info->fGain;
            frame_info_msg->f_exposure_time = frame_info->fExposureTime;
            frame_info_msg->n_average_brightness = frame_info->nAverageBrightness;
            frame_info_msg->n_red = frame_info->nRed;
            frame_info_msg->n_green = frame_info->nGreen;
            frame_info_msg->n_blue = frame_info->nBlue;
            frame_info_msg->n_frame_counter = frame_info->nFrameCounter;
            frame_info_msg->n_trigger_index = frame_info->nTriggerIndex;
            frame_info_msg->n_input = frame_info->nInput;
            frame_info_msg->n_output = frame_info->nOutput;
            frame_info_msg->n_offset_x = frame_info->nOffsetX;
            frame_info_msg->n_offset_y = frame_info->nOffsetY;
            frame_info_msg->n_chunk_width = frame_info->nChunkWidth;
            frame_info_msg->n_chunk_height = frame_info->nChunkHeight;
            frame_info_msg->n_lost_packet = frame_info->nLostPacket;
            
            // 处理未解析的Chunk数据
            // 注意：根据实际SDK结构体定义调整字段名
            frame_info_msg->n_unparsed_chunk_num = frame_info->nUnparsedChunkNum;
            if (frame_info->nUnparsedChunkNum > 0)
            {
                frame_info_msg->unparsed_chunk_content.resize(frame_info->nUnparsedChunkNum);
                for(unsigned int i = 0; i < frame_info->nUnparsedChunkNum; i++)
                {
                    // 透传Chunk 信息
                    // 注释掉Chunk数据处理，因为某些SDK版本中MV_FRAME_OUT_INFO_EX结构体可能没有stChunkInfo成员
                    
                    frame_info_msg->unparsed_chunk_content[i].p_chunk_data.resize(frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkLen);
                    memcpy(frame_info_msg->unparsed_chunk_content[i].p_chunk_data.data(), 
                           frame_info->UnparsedChunkList.pUnparsedChunkContent[i].pChunkData, 
                           frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkLen);
                    frame_info_msg->unparsed_chunk_content[i].n_chunk_id = frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkID;
                    frame_info_msg->unparsed_chunk_content[i].n_chunk_len = frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkLen;
                    
                }
            }
            
            // 发布FrameInfo消息
            frame_info_pub_->publish(*frame_info_msg);
        }
        
        logPublishSummary(
            frame_info->nFrameNum,
            convert_info.nWidth,
            convert_info.nHeight,
            static_cast<uint32_t>(convert_info.enDestPixelType));
        seq_++;
    }

    void onNewProcessedFrameReport(unsigned char * pData, MV_FRAME_OUT_INFO_EX* frame_info)
    {


        // 在这里创建所有的消息
        // 创建智能指针
        sensor_msgs::msg::Image::SharedPtr image_ptr;
        auto info_ptr = std::make_shared<sensor_msgs::msg::CameraInfo>();
        
        // 设置 Header
        std_msgs::msg::Header header;
        header.stamp = this->now();  // 或从相机获取精确时间戳  
        
        // 根据像素格式创建图像消息
        if(frame_info->enPixelType == PixelType_Gvsp_Mono8)
        {
            header.frame_id =  "camera_link";           //普通单目灰度
            // 创建 Mat（引用数据）
            cv::Mat stImgTmp = cv::Mat(cv::Size(frame_info->nExtendWidth, 
                                    frame_info->nExtendHeight), 
                                    CV_8UC1, 
                                    pData);

            // 转为 ROS 2 消息（直接使用智能指针，避免拷贝）
            image_ptr = cv_bridge::CvImage(header, "mono8", stImgTmp).toImageMsg();


        }
        else if(frame_info->enPixelType == PixelType_Gvsp_RGB8_Packed)
        {
            header.frame_id = "camera_color_frame";           //普通 彩色相机
            // 创建 Mat（引用数据）
            cv::Mat stImgTmp = cv::Mat(cv::Size(frame_info->nExtendWidth,
                                    frame_info->nExtendHeight), 
                                    CV_8UC3, 
                                    pData);
            // 转为 ROS 2 消息（直接使用智能指针，避免拷贝）
            image_ptr = cv_bridge::CvImage(header, "rgb8", stImgTmp).toImageMsg();
        }
        else
        {
            // 不支持的像素格式
            RCLCPP_ERROR(this->get_logger(), "Not support pixeltype  [%#x] .", static_cast<unsigned int>(frame_info->enPixelType));
            return;
        }
        
        // === 2. 创建 CameraInfo 消息 ===
        info_ptr->header = header;
        info_ptr->width = frame_info->nExtendWidth;
        info_ptr->height = frame_info->nExtendHeight;

        // === 发布标准 消息 ===
        image_pub_->publish(*image_ptr);
        info_pub_->publish(*info_ptr);

        // 创建并发布FrameInfo消息
        if(frame_info_pub_ && frame_info_pub_->get_subscription_count() > 0)
        {
            auto frame_info_msg = std::make_shared<mvcc_camera_ros2_interface::msg::FrameInfo>();
            
            // 填充FrameInfo消息
            frame_info_msg->image = *image_ptr;
            frame_info_msg->header = header;
            
            frame_info_msg->n_extend_width = frame_info->nExtendWidth;
            frame_info_msg->n_extend_height = frame_info->nExtendHeight;
            frame_info_msg->n_frame_len_ex = frame_info->nFrameLenEx;
            frame_info_msg->en_pixel_type = static_cast<uint32_t>(frame_info->enPixelType);
            frame_info_msg->n_frame_num = frame_info->nFrameNum;
            frame_info_msg->n_dev_timestamp_high = frame_info->nDevTimeStampHigh;
            frame_info_msg->n_dev_timestamp_low = frame_info->nDevTimeStampLow;
            frame_info_msg->n_host_timestamp = frame_info->nHostTimeStamp;
            frame_info_msg->n_second_count = frame_info->nSecondCount;
            frame_info_msg->n_cycle_count = frame_info->nCycleCount;
            frame_info_msg->n_cycle_offset = frame_info->nCycleOffset;
            frame_info_msg->f_gain = frame_info->fGain;
            frame_info_msg->f_exposure_time = frame_info->fExposureTime;
            frame_info_msg->n_average_brightness = frame_info->nAverageBrightness;
            frame_info_msg->n_red = frame_info->nRed;
            frame_info_msg->n_green = frame_info->nGreen;
            frame_info_msg->n_blue = frame_info->nBlue;
            frame_info_msg->n_frame_counter = frame_info->nFrameCounter;
            frame_info_msg->n_trigger_index = frame_info->nTriggerIndex;
            frame_info_msg->n_input = frame_info->nInput;
            frame_info_msg->n_output = frame_info->nOutput;
            frame_info_msg->n_offset_x = frame_info->nOffsetX;
            frame_info_msg->n_offset_y = frame_info->nOffsetY;
            frame_info_msg->n_chunk_width = frame_info->nChunkWidth;
            frame_info_msg->n_chunk_height = frame_info->nChunkHeight;
            frame_info_msg->n_lost_packet = frame_info->nLostPacket;
            
            // 处理未解析的Chunk数据
            // 注意：根据实际SDK结构体定义调整字段名
            frame_info_msg->n_unparsed_chunk_num = frame_info->nUnparsedChunkNum;
            if (frame_info->nUnparsedChunkNum > 0)
            {
                frame_info_msg->unparsed_chunk_content.resize(frame_info->nUnparsedChunkNum);
                for(unsigned int i = 0; i < frame_info->nUnparsedChunkNum; i++)
                {
                    // 透传Chunk 信息
                    // 注释掉Chunk数据处理，因为某些SDK版本中MV_FRAME_OUT_INFO_EX结构体可能没有stChunkInfo成员
                    
                    frame_info_msg->unparsed_chunk_content[i].p_chunk_data.resize(frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkLen);
                    memcpy(frame_info_msg->unparsed_chunk_content[i].p_chunk_data.data(), 
                           frame_info->UnparsedChunkList.pUnparsedChunkContent[i].pChunkData, 
                           frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkLen);
                    frame_info_msg->unparsed_chunk_content[i].n_chunk_id = frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkID;
                    frame_info_msg->unparsed_chunk_content[i].n_chunk_len = frame_info->UnparsedChunkList.pUnparsedChunkContent[i].nChunkLen;
                    
                }
            }
            
            // 发布FrameInfo消息
            frame_info_pub_->publish(*frame_info_msg);
        }
        
        logPublishSummary(
            frame_info->nFrameNum,
            frame_info->nExtendWidth,
            frame_info->nExtendHeight,
            static_cast<uint32_t>(frame_info->enPixelType));
        seq_++;
    }

private:
    void logPublishSummary(
        uint32_t frame_num,
        uint32_t width,
        uint32_t height,
        uint32_t pixel_type)
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "[Publisher] streaming: published=%u frame=%u size=%ux%u pixel=0x%x",
            seq_ + 1,
            frame_num,
            width,
            height,
            pixel_type);
    }

    HikCamera camera_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    rclcpp::Publisher<mvcc_camera_ros2_interface::msg::FrameInfo>::SharedPtr frame_info_pub_;
    
    // 相机参数服务管理器
    std::unique_ptr<CameraParameterServiceManager> parameter_service_manager_;

    string serial_number_;
    uint32_t seq_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
     {
        auto node = std::make_shared<ImagePublish>();
        rclcpp::spin(node);
        
    } catch (const std::exception& e) 
    {
        // 捕获构造函数中抛出的异常
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to create ImagePublish node: %s", e.what());
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::shutdown();
    return 0;
}
