#include "../common/hik_camera.h"



// 添加字符串处理头文件
#include <algorithm>
#include <cctype>
#include <cstring>
#include <chrono>
#include <iostream>
#include <cstdio>
#include <sys/stat.h>
#include <sys/types.h>
#ifdef _WIN32
#include <direct.h>
#include <io.h>
#define access _access
#define mkdir(path) _mkdir(path)
#else
#include <unistd.h>
#endif

// ROS2日志系统头文件
#include <rclcpp/rclcpp.hpp>



#define OPERATION_PRINT                      1   //打印
#define OPERATION_UPDATE_SERIAL              2   //更新序列号



// 日志宏定义
#define CAMERA_LOG_INFO(format, ...) \
    RCLCPP_INFO(rclcpp::get_logger("HikCamera"), "[%s] " format, __FUNCTION__, ##__VA_ARGS__)

#define CAMERA_LOG_DEBUG(format, ...) \
    RCLCPP_DEBUG(rclcpp::get_logger("HikCamera"), "[%s] " format, __FUNCTION__, ##__VA_ARGS__)

#define CAMERA_LOG_WARN(format, ...) \
    RCLCPP_WARN(rclcpp::get_logger("HikCamera"), "[%s] " format, __FUNCTION__, ##__VA_ARGS__)

#define CAMERA_LOG_ERROR(format, ...) \
    RCLCPP_ERROR(rclcpp::get_logger("HikCamera"), "[%s:%d] " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)

namespace {

bool isInternalRosParameter(const std::string& param_name)
{
    return param_name == "cam_sn" ||
           param_name == "use_sim_time" ||
           param_name.rfind("qos_overrides.", 0) == 0;
}

}  // namespace


    
void  HikCamera::DeviceInfoProc(MV_CC_DEVICE_INFO* pstMVDevInfo, unsigned int nOperation)
{
    if (NULL == pstMVDevInfo)
    {
        CAMERA_LOG_ERROR("The Pointer of pstMVDevInfo is NULL!");
        return ;
    }

    string strUserDefinedName = "";
    string strSerialNumber = "";
    string strModelName = "";

    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        strUserDefinedName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        strSerialNumber = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        strModelName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        strUserDefinedName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        strSerialNumber = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        strModelName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);        
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
    {
        strUserDefinedName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
        strSerialNumber = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        strModelName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
    {
        strUserDefinedName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCMLInfo.chUserDefinedName);
        strSerialNumber = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
        strModelName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCMLInfo.chModelName);       
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
    {
        strUserDefinedName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCXPInfo.chUserDefinedName);
        strSerialNumber = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
        strModelName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCXPInfo.chModelName);      
    }
    else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
    {
        strUserDefinedName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stXoFInfo.chUserDefinedName);
        strSerialNumber = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
        strModelName = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stXoFInfo.chModelName);           
    }
    else 
    {
        CAMERA_LOG_ERROR("Not support nTLayerType [%d]", pstMVDevInfo->nTLayerType);
        return;
    }

    if(OPERATION_UPDATE_SERIAL == nOperation)
    {
        // 从设备信息中提取序列号
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            serial_number_ = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            serial_number_ = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_GIGE_DEVICE)
        {
            serial_number_ = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_CAMERALINK_DEVICE)
        {
            serial_number_ = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_CXP_DEVICE)
        {
            serial_number_ = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber);
        }
        else if (pstMVDevInfo->nTLayerType == MV_GENTL_XOF_DEVICE)
        {
            serial_number_ = reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber);
        }
        return ;
    }
    else if(OPERATION_PRINT == nOperation)
    {
        CAMERA_LOG_INFO("UserDefinedName = %s", strUserDefinedName.c_str());
        CAMERA_LOG_INFO("SerialNumber = %s", strSerialNumber.c_str());
        CAMERA_LOG_INFO("ModelName = %s", strModelName.c_str());
    }
    else
    {
        CAMERA_LOG_ERROR("Not support nOperation [%d]", nOperation);
        return ;
    }


    return ;
}

bool  HikCamera::findDevIndexBySerial(const char* pSerial,  MV_CC_DEVICE_INFO_LIST* pstMVDevInfoList, unsigned int* pnIndex)
{   
    bool bFind = false;

    for (unsigned int i = 0; i < pstMVDevInfoList->nDeviceNum; i++)
    {
        MV_CC_DEVICE_INFO* pstMVDevInfo = pstMVDevInfoList->pDeviceInfo[i];
        if (NULL == pstMVDevInfo)
        {
            CAMERA_LOG_ERROR("The Pointer of pstMVDevInfo is NULL");
            return false;
        } 


        if (MV_GIGE_DEVICE == pstMVDevInfo->nTLayerType)
        {
            if(0 == strncmp(reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber), pSerial, strlen(pSerial)))
            {
                bFind = true;
                *pnIndex = i;
                break;
            }
        }
        else if (MV_USB_DEVICE == pstMVDevInfo->nTLayerType)
        {
            if(0 == strncmp(reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber), pSerial, strlen(pSerial)))
            {
                bFind = true;
                *pnIndex = i;
                break;
            }
        }
        else if (MV_GENTL_GIGE_DEVICE == pstMVDevInfo->nTLayerType)
        {
            if(0 == strncmp(reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber), pSerial, strlen(pSerial)))
            {
                bFind = true;
                *pnIndex = i;
                break;
            }
        }
        else if (MV_GENTL_CAMERALINK_DEVICE == pstMVDevInfo->nTLayerType)
        {
            if(0 == strncmp(reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCMLInfo.chSerialNumber), pSerial, strlen(pSerial)))
            {
                bFind = true;
                *pnIndex = i;
                break;
            }
        }
        else if (MV_GENTL_CXP_DEVICE == pstMVDevInfo->nTLayerType)
        {
            if(0 == strncmp(reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stCXPInfo.chSerialNumber), pSerial, strlen(pSerial)))
            {
                bFind = true;
                *pnIndex = i;
                break;
            }
        }
        else if (MV_GENTL_XOF_DEVICE == pstMVDevInfo->nTLayerType)
        {
            if(0 == strncmp(reinterpret_cast<const char*>(pstMVDevInfo->SpecialInfo.stXoFInfo.chSerialNumber), pSerial, strlen(pSerial)))
            {
                bFind = true;
                *pnIndex = i;
                break;
            }
        }
        else
        {
            CAMERA_LOG_ERROR("Not support nTLayerType [%d]", pstMVDevInfo->nTLayerType);
            continue;
        }
    }  

    return bFind;
}



HikCamera::HikCamera()
{
    CAMERA_LOG_INFO("HikCamera constructor");

    initialize();

    return ;
}

HikCamera::~HikCamera()
{
    CAMERA_LOG_INFO("HikCamera destructor begin");
    
    // 使用互斥锁保护析构过程
    std::lock_guard<std::mutex> hb_lock(hb_buffer_mutex_);
    std::lock_guard<std::mutex> convert_lock(convert_buffer_mutex_);
    
    // 先停止取流和关闭设备
    try {
        if (handle_ != nullptr) {
            stop();
            close();
        }
    } catch (const std::exception& e) {
        CAMERA_LOG_ERROR("Exception during cleanup: %s", e.what());
    }
    
    // 清理缓冲区
    if (convert_buffer_) {
        delete[] convert_buffer_;
        convert_buffer_ = nullptr;
    }
    convert_buffer_len_ = 0;

    if (hb_decode_buffer_) {
        delete[] hb_decode_buffer_;
        hb_decode_buffer_ = nullptr;
    }
    hb_decode_buffer_len_ = 0;

    // 清理回调函数
    {
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        extended_image_callback_ = nullptr;
        extended_user_ = nullptr;
    }

    // 最终化SDK
    MV_CC_Finalize();

    CAMERA_LOG_INFO("HikCamera destructor end");
}

int HikCamera::initialize()
{
    CAMERA_LOG_INFO("Initializing camera SDK");
        

    int nRet = MV_CC_Initialize();
    if (MV_OK != nRet) {
        CAMERA_LOG_ERROR("MV_CC_Initialize failed, error code: 0x%x", nRet);
        return nRet;
    }

    // 获取SDK版本信息
    int SDKVersion = MV_CC_GetSDKVersion();
    CAMERA_LOG_INFO("SDK version: 0x%x", SDKVersion);
    
    CAMERA_LOG_INFO("Camera SDK initialized successfully");
    
    return MV_OK;
}

int HikCamera::open(const char* chSerialNumber)
{
    CAMERA_LOG_INFO("Opening camera device");
       
    
    if (handle_ != nullptr) {
        CAMERA_LOG_WARN("Camera already opened");
        return MV_OK;
    }

    // 枚举设备
    MV_CC_DEVICE_INFO_LIST stDeviceList;
    memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE | MV_GENTL_CAMERALINK_DEVICE | 
                                MV_GENTL_CXP_DEVICE | MV_GENTL_XOF_DEVICE, &stDeviceList);
    if (MV_OK != nRet) {
        CAMERA_LOG_ERROR("MV_CC_EnumDevices failed, error code: 0x%x", nRet);
        return nRet;
    }

    if (stDeviceList.nDeviceNum == 0) {
        CAMERA_LOG_ERROR("No devices found");
        return -1;
    }

    // 打印所有设备信息
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
        MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
        if (pDeviceInfo != nullptr) {
            DeviceInfoProc(pDeviceInfo, OPERATION_PRINT);
        }
    }

    // 选择设备
    unsigned int nChoiceIndex = 0;
    if (chSerialNumber == nullptr) {
        // 连接第一个设备
        nChoiceIndex = 0;
        CAMERA_LOG_INFO("No serial number specified, connecting to first device (index %d)", nChoiceIndex);
    } else {
        // 处理序列号前缀
        const char* processedSerialNumber = chSerialNumber;
        std::string serialNumberStr;
        
         // 特殊情况支持，如果是const char* chSerialNumber 是'string:00775075482' 的话，去掉 'string:'  将 '00775075482' 传入  
        std::string inputSerial(chSerialNumber);
        if (inputSerial.substr(0, 7) == "string:") {
            serialNumberStr = inputSerial.substr(7);
            processedSerialNumber = serialNumberStr.c_str();
            CAMERA_LOG_INFO("Removed 'string:' prefix. Original: [%s], Processed: [%s]", 
                           chSerialNumber, processedSerialNumber);
        }
        
        // 查找指定序列号的设备
        bool bFindDevice = findDevIndexBySerial(processedSerialNumber, &stDeviceList, &nChoiceIndex);
        if (!bFindDevice) {
            CAMERA_LOG_ERROR("Device with serial number [%s] not found", processedSerialNumber);
            return -1;
        }
        CAMERA_LOG_INFO("Found device with serial number [%s] at index [%d]", 
                       processedSerialNumber, nChoiceIndex);
    }

    // 创建设备句柄
    nRet = MV_CC_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nChoiceIndex]);
    if (MV_OK != nRet) {
        CAMERA_LOG_ERROR("MV_CC_CreateHandle failed, error code: 0x%x", nRet);
        return nRet;
    }

    // 打开设备
    nRet = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet) {
        CAMERA_LOG_ERROR("MV_CC_OpenDevice failed, error code: 0x%x", nRet);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
        return nRet;
    }

    // 设置图像节点数量
    nRet = MV_CC_SetImageNodeNum(handle_, DEFAULT_QUEUE_SIZE);
    if (MV_OK != nRet) {
        CAMERA_LOG_WARN("Warning: Set Image Node Num failed, error code: 0x%x", nRet);
        // 配置节点失败不一定异常，继续执行
    } else {
        CAMERA_LOG_INFO("Set Image Node Num [%d] success", DEFAULT_QUEUE_SIZE);
    }

    // 网口相机优化包大小
    if (stDeviceList.pDeviceInfo[nChoiceIndex]->nTLayerType == MV_GIGE_DEVICE) {
        int nPacketSize = MV_CC_GetOptimalPacketSize(handle_);
        if (nPacketSize > 0) {
            nRet = MV_CC_SetIntValueEx(handle_, "GevSCPSPacketSize", nPacketSize);
            if (nRet != MV_OK) {
                CAMERA_LOG_WARN("Warning: Set Packet Size failed, error code: 0x%x", nRet);
            } else {
                CAMERA_LOG_INFO("Set optimal packet size to %d", nPacketSize);
            }
        } else {
            CAMERA_LOG_WARN("Warning: Get optimal packet size failed, return code: %d", nPacketSize);
        }
    }

    // 记录序列号
    DeviceInfoProc(stDeviceList.pDeviceInfo[nChoiceIndex], OPERATION_UPDATE_SERIAL);

    CAMERA_LOG_INFO("Successfully opened device index [%d] with serial number [%s]", 
                   nChoiceIndex, serial_number_.c_str());
    
    return MV_OK;
}



int HikCamera::start()
{
    CAMERA_LOG_INFO("start begin"); 

    if (handle_ == nullptr) {
        CAMERA_LOG_WARN("Camera not opened");
        return MV_E_PRECONDITION;
    }

    int nRet = MV_OK;
    nRet = MV_CC_StartGrabbing(handle_);
    if (MV_OK != nRet) 
    {
        CAMERA_LOG_ERROR("MV_CC_StartGrabbing failed...sts[%#x]", nRet);
        return nRet;
    }

    CAMERA_LOG_INFO("start end"); 

    return nRet;
}


void HikCamera::stop()
{
    CAMERA_LOG_INFO("stop begin"); 
    if (handle_ != nullptr) 
    {
        int nRet = MV_CC_StopGrabbing(handle_);
        if (MV_OK != nRet) 
        {
            CAMERA_LOG_ERROR("MV_CC_StopGrabbing failed...sts[%#x]", nRet);
        }
    }

    CAMERA_LOG_INFO("stop end"); 
}



void HikCamera::close()
{
    CAMERA_LOG_INFO("close begin"); 

    if (handle_ != nullptr)
     {
        // 先停止取流
        MV_CC_StopGrabbing(handle_);
        
        // 关闭设备
        int nRet = MV_CC_CloseDevice(handle_);
        if (MV_OK != nRet) 
        {
            CAMERA_LOG_ERROR("MV_CC_CloseDevice failed...sts[%#x]", nRet);
        }
        
        MV_CC_DestroyHandle(handle_);
        // 重置句柄
        handle_ = nullptr;
    }

    CAMERA_LOG_INFO("close end"); 
}

int HikCamera::getImageBuffer( MV_FRAME_OUT* pstFrame,  unsigned int nMsec)
{
    int nRet = MV_OK;
    memset(pstFrame, 0, sizeof(MV_FRAME_OUT));
    CAMERA_LOG_INFO("GetImageBuffer begin"); 
    nRet = MV_CC_GetImageBuffer(handle_, pstFrame, nMsec);
     if (nRet != MV_OK)
    {
        CAMERA_LOG_ERROR("MV_CC_GetImageBuffer failed, nRet [%x]", nRet);
        return nRet;
    }
     CAMERA_LOG_INFO("MV_CC_GetImageBuffer success: Width[%d], Height[%d], nFrameNum[%d]",
          pstFrame->stFrameInfo.nExtendWidth, pstFrame->stFrameInfo.nExtendHeight, pstFrame->stFrameInfo.nFrameNum);

    return nRet;
}


int HikCamera::freeImageBuffer(MV_FRAME_OUT* pstFrame)
{
    int nRet = MV_OK;
    nRet = MV_CC_FreeImageBuffer(handle_, pstFrame);
    if (MV_OK != nRet)
    {
        CAMERA_LOG_ERROR("MV_CC_FreeImageBuffer failed, nRet [%x]", nRet);
    }

    return nRet;
}


//内部回调函数
void __stdcall  HikCamera::Inner_ImageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser)
{
    if (pUser)
    {
        HikCamera* pHikCamera = (HikCamera*)pUser;
        if(pHikCamera)
        {
            pHikCamera->InnerImageCallbackProc(pData,pFrameInfo);
        }

    }
}



// 内部回调 
void HikCamera::InnerImageCallbackProc(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo)
{

    int nRet = MV_OK;

    CAMERA_LOG_DEBUG(
        "Recv Image Frame Number [%u] nExtendWidth[%u] nExtendHeight[%u] nFrameLenEx[%llu] nFrameLen[%u] enPixelType[%#x]",
        static_cast<unsigned int>(pFrameInfo->nFrameNum),
        static_cast<unsigned int>(pFrameInfo->nExtendWidth),
        static_cast<unsigned int>(pFrameInfo->nExtendHeight),
        static_cast<unsigned long long>(pFrameInfo->nFrameLenEx),
        static_cast<unsigned int>(pFrameInfo->nFrameLen),
        static_cast<unsigned int>(pFrameInfo->enPixelType));


    MVCC_ROS2_IMAGE_PROC_OUT stOut;
    // 图像处理耗时，后期根据实际需求优化
    // 方法1. 扩大SDK缓存节点;  MV_CC_SetImageNodeNum
    // 方法2. 或者不需要转码的话，修改.msg 直接传递原始图
    nRet = imageDecodeAndConvertProcess(pData, pFrameInfo, &stOut);
    if(MV_OK != nRet)
    {
        CAMERA_LOG_ERROR("InnerImageCallbackProc imageDecodeAndConvertProcess nRet [%x]", nRet);
        return;
    }
    
    CAMERA_LOG_DEBUG("InnerImageCallbackProc imageDecodeAndConvertProcess success");

    // 如果有扩展回调函数，则调用扩展回调函数
    {
        std::lock_guard<std::mutex> callback_lock(callback_mutex_);
        if(extended_image_callback_)
        {
            // 创建MV_FRAME_OUT结构体
            MV_FRAME_OUT_INFO_EX frameOut;
            memset(&frameOut, 0, sizeof(MV_FRAME_OUT_INFO_EX));

            frameOut = *pFrameInfo;
            
            
            //更新图像信息 （图像可能在OriImageProc中进行了必要的转换)
            frameOut.enPixelType = stOut.enPixelType;

            if(stOut.nWidth <= std::numeric_limits<unsigned short>::max())
            {
                frameOut.nWidth = stOut.nWidth;
            }
            if(stOut.nHeight <= std::numeric_limits<unsigned short>::max())
            {
                frameOut.nHeight = stOut.nHeight;
            }

            frameOut.nExtendWidth = stOut.nWidth;
            frameOut.nExtendHeight = stOut.nHeight;

            if(stOut.nBufLen <= std::numeric_limits<unsigned int>::max())
            {
                frameOut.nFrameLen = stOut.nBufLen;
            }

            frameOut.nFrameLenEx = stOut.nBufLen;

            CAMERA_LOG_DEBUG("extended_image_callback_ begin");
            // 调用扩展回调函数，传入帧信息
            extended_image_callback_(stOut.pBufAddr, &frameOut, extended_user_);

            CAMERA_LOG_DEBUG("extended_image_callback_ end");
        }
    }
}


    // 回调方式1: 内部图像处理后回调输出
int HikCamera::registerProcessedImageCallback(MvImageCallback_ros2 cbImageCallback, void* pUser)
{
    std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    
    int nRet = MV_OK;

    if (cbImageCallback == nullptr)
    {
        nRet = MV_CC_RegisterImageCallBackEx(handle_, nullptr, nullptr);
    }
    else
    {
        nRet = MV_CC_RegisterImageCallBackEx(handle_, Inner_ImageCallback, this);
    }

    if (nRet == MV_OK)
    {
        extended_image_callback_ = cbImageCallback;
        extended_user_ = pUser;
    }
    else
    {
        CAMERA_LOG_ERROR("Failed to register extended image callback, error: 0x%x", nRet);
    }

    return nRet;
}

    // 回调方式2:  直接上报相机输出的图像到应用层
    int HikCamera::registerImageCallback(MvImageCallback_ros2 cbImageCallback, void* pUser)
    {
        int nRet = MV_OK;
        if (cbImageCallback == nullptr)
        {
            nRet = MV_CC_RegisterImageCallBackEx(handle_, nullptr, pUser);
        }
        else
        {
            nRet = MV_CC_RegisterImageCallBackEx(handle_, cbImageCallback, pUser);
        }

        return nRet;
    }



int HikCamera::setCameraParameter(const char* chNode, MVCC_ROS2_NODE_Param* pstNodeParam)
{
    if (handle_ == nullptr) {
        CAMERA_LOG_ERROR("Camera not connected, cannot set parameters");
        return MV_E_HANDLE;
    }

    if (chNode == nullptr || pstNodeParam == nullptr) {
        CAMERA_LOG_ERROR("Invalid parameter");
        return MV_E_PARAMETER;
    }

    int nRet = MV_OK;

    // 根据参数类型调用相应的SDK函数
    switch (pstNodeParam->type)
    {
        case MVCC_ROS2_NODE_INT:
        {
            // 获取当前参数值范围
            MVCC_INTVALUE_EX stIntValue;
            nRet = MV_CC_GetIntValueEx(handle_, chNode, &stIntValue);
            if (MV_OK != nRet) {
                CAMERA_LOG_ERROR("Failed to get int parameter [%s] info, error: 0x%x", chNode, nRet);
                return nRet;
            }

            CAMERA_LOG_DEBUG("Current [%s]: %ld, Min: %ld, Max: %ld", 
                       chNode, stIntValue.nCurValue, stIntValue.nMin, stIntValue.nMax);

            // 检查参数值是否在有效范围内
            if (pstNodeParam->value.intValue >= stIntValue.nMin && pstNodeParam->value.intValue <= stIntValue.nMax)
             {
            nRet = MV_CC_SetIntValueEx(handle_, chNode, pstNodeParam->value.intValue);
            if (MV_OK == nRet) 
            {
                CAMERA_LOG_DEBUG("Set [%s] to %lld successfully", chNode, (long long)pstNodeParam->value.intValue);
            } 
            else 
            {
                CAMERA_LOG_ERROR("Failed to set [%s] to %lld, error: 0x%x", chNode, (long long)pstNodeParam->value.intValue, nRet);
            }
            } 
            else
             {
                CAMERA_LOG_WARN("[%s] value %lld is out of range [%ld, %ld]", 
                           chNode, (long long)pstNodeParam->value.intValue, stIntValue.nMin, stIntValue.nMax);
                return MV_E_PARAMETER;
            }
            break;
        }
        case MVCC_ROS2_NODE_FLOAT:
        {
            // 获取当前参数值范围
            MVCC_FLOATVALUE stFloatValue;
            nRet = MV_CC_GetFloatValue(handle_, chNode, &stFloatValue);
            if (MV_OK != nRet) 
            {
                CAMERA_LOG_ERROR("Failed to get float parameter [%s] info, error: 0x%x", chNode, nRet);
                return nRet;
            }

            CAMERA_LOG_DEBUG("Current [%s]: %f, Min: %f, Max: %f", 
                       chNode, stFloatValue.fCurValue, stFloatValue.fMin, stFloatValue.fMax);

            // 检查参数值是否在有效范围内
            if (pstNodeParam->value.floatValue >= stFloatValue.fMin && pstNodeParam->value.floatValue <= stFloatValue.fMax) 
            {
                nRet = MV_CC_SetFloatValue(handle_, chNode, pstNodeParam->value.floatValue);
                if (MV_OK == nRet) 
                {
                    CAMERA_LOG_DEBUG("Set [%s] to %f successfully", chNode, pstNodeParam->value.floatValue);
                } 
                else
                 {
                    CAMERA_LOG_ERROR("Failed to set [%s] to %f, error: %d", chNode, pstNodeParam->value.floatValue, nRet);
                }
            } 
            else 
            {
                CAMERA_LOG_WARN("[%s] value %f is out of range [%f, %f]", 
                           chNode, pstNodeParam->value.floatValue, stFloatValue.fMin, stFloatValue.fMax);
                return MV_E_PARAMETER;
            }
            break;
        }
        case MVCC_ROS2_NODE_BOOL:
        {
            nRet = MV_CC_SetBoolValue(handle_, chNode, pstNodeParam->value.boolValue);
            if (MV_OK == nRet) 
            {
                CAMERA_LOG_DEBUG("Set [%s] to %s successfully", 
                           chNode, pstNodeParam->value.boolValue ? "true" : "false");
            } 
            else
            {
                CAMERA_LOG_ERROR("Failed to set [%s] to %s, error: %d", 
                            chNode, pstNodeParam->value.boolValue ? "true" : "false", nRet);
            }
            break;
        }
        case MVCC_ROS2_NODE_COMMAND:
        {
            nRet = MV_CC_SetCommandValue(handle_, chNode);
            if (MV_OK == nRet) 
            {
                CAMERA_LOG_DEBUG("Execute [%s] command successfully", chNode);
            } 
            else
            {
                CAMERA_LOG_ERROR("Failed to execute [%s] command, error: %d", chNode, nRet);
            }
            break;
        }
        case MVCC_ROS2_NODE_STRING:
        {
            nRet = MV_CC_SetStringValue(handle_, chNode, pstNodeParam->value.szStringValue);
            if (MV_OK == nRet) 
            {
                CAMERA_LOG_DEBUG("Set [%s] to %s successfully", 
                           chNode, pstNodeParam->value.szStringValue);
            } 
            else
            {
                CAMERA_LOG_ERROR("Failed to set [%s] to %s, error: %d", 
                            chNode, pstNodeParam->value.szStringValue, nRet);
            }
            break;
        }
        case MVCC_ROS2_NODE_ENUM:
        {
            nRet = MV_CC_SetEnumValueByString(handle_, chNode, pstNodeParam->value.szStringValue);
            if (MV_OK == nRet) 
            {
                CAMERA_LOG_DEBUG("Set [%s] to %s successfully", 
                           chNode, pstNodeParam->value.szStringValue);
            } 
            else
            {
                CAMERA_LOG_ERROR("Failed to set [%s] to %s, error: %d", 
                            chNode, pstNodeParam->value.szStringValue, nRet);
            }
            break;
        }
        default:
        {
            CAMERA_LOG_ERROR("Unsupported parameter type: %d", pstNodeParam->type);
            return MV_E_PARAMETER;
        }
    }

    return nRet;
}


int HikCamera::getCameraParameter(const char* chNode, MVCC_ROS2_NODE_Param* pstNodeParam)
{
    if (handle_ == nullptr) {
        CAMERA_LOG_ERROR("Camera not connected, cannot set parameters");
        return MV_E_HANDLE;
    }

    if (chNode == nullptr || pstNodeParam == nullptr) {
        CAMERA_LOG_ERROR("Invalid parameter");
        return MV_E_PARAMETER;
    }

    int nRet = MV_OK;

    // 根据参数类型调用相应的SDK函数
    switch (pstNodeParam->type)
    {
        case MVCC_ROS2_NODE_INT:
        {
            // 获取当前参数值
            MVCC_INTVALUE_EX stIntValue;
            nRet = MV_CC_GetIntValueEx(handle_, chNode, &stIntValue);
            if (MV_OK != nRet) {
                CAMERA_LOG_ERROR("Failed to get int parameter [%s] info, error: 0x%x", chNode, nRet);
                return nRet;
            }

            CAMERA_LOG_INFO("Current [%s]: %ld, Min: %ld, Max: %ld", 
                       chNode, stIntValue.nCurValue, stIntValue.nMin, stIntValue.nMax);

            pstNodeParam->value.intValue = stIntValue.nCurValue;    
            break;
        }
        case MVCC_ROS2_NODE_FLOAT:
        {
            // 获取当前参数值
            MVCC_FLOATVALUE stFloatValue;
            nRet = MV_CC_GetFloatValue(handle_, chNode, &stFloatValue);
            if (MV_OK != nRet) 
            {
                CAMERA_LOG_ERROR("Failed to get float parameter [%s] info, error: 0x%x", chNode, nRet);
                return nRet;
            }

            CAMERA_LOG_INFO("Current [%s]: %f, Min: %f, Max: %f", 
                       chNode, stFloatValue.fCurValue, stFloatValue.fMin, stFloatValue.fMax);

             pstNodeParam->value.floatValue = stFloatValue.fCurValue;
                       
            break;
        }
        case MVCC_ROS2_NODE_BOOL:
        {
            bool bvalue = false;
            nRet = MV_CC_GetBoolValue(handle_, chNode, &bvalue);
            if (MV_OK == nRet) 
            {
                CAMERA_LOG_INFO("Set [%s] to %s successfully", 
                           chNode, pstNodeParam->value.boolValue ? "true" : "false");
            } 
            else
            {
                CAMERA_LOG_ERROR("Failed to set [%s] to %s, error: %d", 
                            chNode, pstNodeParam->value.boolValue ? "true" : "false", nRet);
            }

            pstNodeParam->value.boolValue = bvalue;
            break;
        }
        case MVCC_ROS2_NODE_STRING:
        {
            MVCC_STRINGVALUE stringvalue;
            nRet = MV_CC_GetStringValue(handle_, chNode, &stringvalue);
            if (MV_OK == nRet) 
            {
                CAMERA_LOG_INFO("Set [%s] to %s successfully", 
                           chNode, pstNodeParam->value.szStringValue);
            } 
            else
            {
                CAMERA_LOG_ERROR("Failed to set [%s] to %s, error: %d", 
                            chNode, pstNodeParam->value.szStringValue, nRet);
            }
            
            memcpy(pstNodeParam->value.szStringValue, stringvalue.chCurValue, strlen(stringvalue.chCurValue));
            break;
        }
        case MVCC_ROS2_NODE_ENUM:
        {
            
            // 调用SDK函数获取枚举型参数值
            MVCC_ENUMVALUE stEnumValue;
            int nRet = MV_CC_GetEnumValue(handle_, chNode, &stEnumValue);
            
            if (MV_OK == nRet) 
            {
                MVCC_ENUMENTRY stEnumEntry;
                memset(&stEnumEntry, 0, sizeof(MVCC_ENUMENTRY));
                stEnumEntry.nValue = stEnumValue.nCurValue;
                nRet = MV_CC_GetEnumEntrySymbolic(handle_, chNode,  &stEnumEntry);
                if (MV_OK == nRet)
                {
                    memcpy(pstNodeParam->value.szStringValue, stEnumEntry.chSymbolic, strlen(stEnumEntry.chSymbolic));

                    CAMERA_LOG_INFO("Get [%s] value: %d, string value: %s", 
                    chNode, stEnumValue.nCurValue, stEnumEntry.chSymbolic);
                }
                else
                {
                    CAMERA_LOG_ERROR("Get [%s] value: %d,  Symbolic failed [%#x].", 
                    chNode, stEnumValue.nCurValue, nRet);
                }
            } 
            else
            {
                CAMERA_LOG_ERROR("Failed to set [%s] to %s, error: %d", 
                            chNode, pstNodeParam->value.szStringValue, nRet);
            }

            break;
        }
        default:
        {
            CAMERA_LOG_ERROR("Unsupported parameter type: %d", pstNodeParam->type);
            return MV_E_PARAMETER;
        }
    }

    return nRet;
}



// 字符串到布尔值的转换函数
bool string_to_bool(const std::string& str) {
    // 转换为小写进行比较
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
    
    // 检查常见的表示true的字符串
    if (lower_str == "true" || lower_str == "1" || lower_str == "yes" || lower_str == "on") {
        return true;
    }
    // 检查常见的表示false的字符串
    else if (lower_str == "false" || lower_str == "0" || lower_str == "no" || lower_str == "off") {
        return false;
    }
    // 默认返回false
    else {
        return false;
    }
}

int HikCamera::setCameraParameterbyString(const std::string& param_name, const std::string& value)
{
    if (param_name.empty()) {
        CAMERA_LOG_ERROR("param_name is empty, cannot configure camera parameters");
        return -1;
    }

    if (value.empty()) {
        CAMERA_LOG_ERROR("value is empty, cannot configure camera parameters");
        return -1;
    }

    if (handle_ == nullptr) {
        CAMERA_LOG_ERROR("Camera handle is null, cannot configure camera parameters");
        return -1;
    }

    int nRet = MV_OK;
    MVCC_ROS2_NODE_Param stParam ;
    
    CAMERA_LOG_DEBUG("setCameraParameterbyString  param_name [%s] begin.",param_name.c_str());

    // 跳过cam_sn参数，因为它在ImagePublish构造函数中已经处理过了
    if (isInternalRosParameter(param_name))
    {
        CAMERA_LOG_DEBUG("Skipping internal ROS parameter [%s].", param_name.c_str());
        return -1;
    }

    // 先使用MV_XML_GetNodeInterfaceType检查参数在相机中是否存在
    MV_XML_InterfaceType interfaceType;
    int xmlRet = MV_XML_GetNodeInterfaceType(handle_, param_name.c_str(), &interfaceType);
    if (MV_OK != xmlRet) {
        // 参数在相机中不存在，跳过
        CAMERA_LOG_DEBUG("Parameter/node [%s] not found in camera, skipping.", param_name.c_str());
        return -1;
    }
    
    // 参数在相机中存在，尝试从ROS参数服务器获取值
    try 
    {
        // 根据 MV_XML_GetNodeInterfaceType 获取的 MV_XML_InterfaceType 类型，把节点值 node->get_parameter(param_name) 按照类型进行转换 
        memset(&stParam, 0, sizeof(MVCC_ROS2_NODE_Param));
        
        switch (interfaceType)
        {
            case IFT_IInteger:
            {
                // 整型参数
                stParam.type = MVCC_ROS2_NODE_INT;
                stParam.value.intValue = std::stoll(value);
                CAMERA_LOG_DEBUG("Parameter [%s] is integer type with value [%lld]", param_name.c_str(), stParam.value.intValue );
                break;
            }
            case IFT_IFloat:
            {
                // 浮点型参数
                stParam.type = MVCC_ROS2_NODE_FLOAT;
                stParam.value.floatValue = std::stof(value);
                CAMERA_LOG_DEBUG("Parameter [%s] is float type with value [%f]", param_name.c_str(), stParam.value.floatValue);
                break;
            }
            case IFT_IBoolean:
            {
                // 布尔型参数
                stParam.type = MVCC_ROS2_NODE_BOOL;
                stParam.value.boolValue = string_to_bool(value);
                CAMERA_LOG_DEBUG("Parameter [%s] is boolean type with value [%s]", param_name.c_str(), stParam.value.boolValue ? "true" : "false");
                break;
            }
            case IFT_IEnumeration:
            {
                 // 枚举参数
                stParam.type = MVCC_ROS2_NODE_ENUM;
                strncpy(stParam.value.szStringValue, value.c_str(), sizeof(stParam.value.szStringValue) - 1);
                stParam.value.szStringValue[sizeof(stParam.value.szStringValue) - 1] = '\0'; // 确保字符串以null结尾
                CAMERA_LOG_DEBUG("Parameter [%s] is enum type with value [%s]", param_name.c_str(), stParam.value.szStringValue);
                break;
            }
            case IFT_IString:
            {
                // 字符串参数
                stParam.type = MVCC_ROS2_NODE_STRING;
                strncpy(stParam.value.szStringValue, value.c_str(), sizeof(stParam.value.szStringValue) - 1);
                stParam.value.szStringValue[sizeof(stParam.value.szStringValue) - 1] = '\0'; // 确保字符串以null结尾
                CAMERA_LOG_DEBUG("Parameter [%s] is string type with value [%s]", param_name.c_str(), stParam.value.szStringValue);
                break;
            }
            default:
            {
                CAMERA_LOG_ERROR("Unsupported parameter type [%d] for parameter [%s], skipping.", interfaceType, param_name.c_str());
                return -1;
            }
        }
        
       
        // 将赋值后的 stParam 调用 setCameraParameter 函数进行设置
        nRet = setCameraParameter(param_name.c_str(), &stParam);
        if(MV_OK != nRet)
        {
            CAMERA_LOG_ERROR("Failed to set parameter [%s], error: [0x%x]", param_name.c_str(), nRet);
            return nRet;
        }
        
        CAMERA_LOG_DEBUG("setCameraParameter [%s] success.", param_name.c_str());
    }
    catch (const std::exception& e) 
    {
        CAMERA_LOG_ERROR("Failed to process parameter [%s]: [%s]", param_name.c_str(), e.what());
        return -1;
    }
    
    CAMERA_LOG_DEBUG("Finished processing camera parameters.");
    return nRet;
}




//方式2： 按照节点类型, 进行操作
int HikCamera::GetIntValue(const char* strKey, MVCC_INTVALUE_EX *pIntValue)
{
    return MV_CC_GetIntValueEx(handle_, strKey, pIntValue);
}

int HikCamera::SetIntValue(const char* strKey, int64_t nValue)
{
    return MV_CC_SetIntValueEx(handle_, strKey, nValue);
}
// ch:获取和设置Enum型参数，如 PixelFormat
int HikCamera::GetEnumValue(const char* strKey, MVCC_ENUMVALUE *pEnumValue)
{
    return MV_CC_GetEnumValue(handle_, strKey, pEnumValue);
}
int HikCamera::SetEnumValue(const char* strKey, unsigned int nValue)
{
    return MV_CC_SetEnumValue(handle_, strKey, nValue); 
}
int HikCamera::SetEnumValueByString(const char* strKey, const char* sValue)
{
    return MV_CC_SetEnumValueByString(handle_, strKey, sValue);
}
int HikCamera::GetEnumEntrySymbolic(const char* strKey, MVCC_ENUMENTRY* pstEnumEntry)
{
    return MV_CC_GetEnumEntrySymbolic(handle_, strKey, pstEnumEntry); 
}

// ch:获取和设置Float型参数，如 ExposureTime和Gain
int HikCamera::GetFloatValue(const char* strKey, MVCC_FLOATVALUE *pFloatValue)
{
    return MV_CC_GetFloatValue(handle_, strKey, pFloatValue);
}
int HikCamera::SetFloatValue(const char* strKey, float fValue)
{
    return MV_CC_SetFloatValue(handle_, strKey, fValue);
}

// ch:获取和设置Bool型参数，如 ReverseX
int HikCamera::GetBoolValue(const char* strKey, bool *pbValue)
{
    return MV_CC_GetBoolValue(handle_, strKey, pbValue);
}
int HikCamera::SetBoolValue(const char* strKey, bool bValue)
{
    return MV_CC_SetBoolValue(handle_, strKey, bValue);
}

// ch:获取和设置String型参数，如 DeviceUserIDint HikCamera::
int HikCamera::GetStringValue(const char* strKey, MVCC_STRINGVALUE *pStringValue)
{
    return MV_CC_GetStringValue(handle_, strKey, pStringValue);
}
int HikCamera::SetStringValue(const char* strKey, const char * strValue)
{
    return MV_CC_SetStringValue(handle_, strKey, strValue);
}

// ch:执行一次Command型命令，如 UserSetSave
int HikCamera::CommandExecute(const char* strKey)
{
    return MV_CC_SetCommandValue(handle_, strKey);  
}


//方式3： 业务专用接口
// 设置曝光时间
int HikCamera::setExposureTime(float fTime)
{
    if (handle_ == nullptr) {
        CAMERA_LOG_ERROR("Camera not connected, cannot set exposure time\n");
        return MV_E_HANDLE;
    }

    // 直接调用SDK函数设置曝光时间
    int nRet = MV_CC_SetFloatValue(handle_, "ExposureTime", fTime);
    
    if (MV_OK == nRet) {
        CAMERA_LOG_ERROR("Set exposure time to %f successfully\n", fTime);
    } else {
        CAMERA_LOG_ERROR("Failed to set exposure time to %f, error: 0x%x\n", fTime, nRet);
    }
    
    return nRet;
}

// 获取曝光时间
int HikCamera::getExposureTime(float* pfTime)
{
    if (handle_ == nullptr) {
        CAMERA_LOG_ERROR("Camera not connected, cannot get exposure time\n");
        return MV_E_HANDLE;
    }

    if (pfTime == nullptr) {
        CAMERA_LOG_ERROR("Invalid parameter: pfTime is null\n");
        return MV_E_PARAMETER;
    }

    // 直接调用SDK函数获取曝光时间
    MVCC_FLOATVALUE stFloatValue;
    int nRet = MV_CC_GetFloatValue(handle_, "ExposureTime", &stFloatValue);
    
    if (MV_OK == nRet) {
        *pfTime = stFloatValue.fCurValue;
        CAMERA_LOG_INFO("Get exposure time: %f\n", *pfTime);
    } else {
        CAMERA_LOG_ERROR("Failed to get exposure time, error: 0x%x\n", nRet);
    }
    
    return nRet;
}



// 图像预处理
int HikCamera::imageDecodeAndConvertProcess(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, MVCC_ROS2_IMAGE_PROC_OUT* pOut)
{
    if (handle_ == nullptr) {
        CAMERA_LOG_ERROR("Camera not connected, cannot set parameters");
        return MV_E_HANDLE;
    }

        //保存图像, 仅存图标识使能后保存
    std::string szSavePath1 = "./picture/" + std::to_string(pFrameInfo->nFrameNum) +
        "Width_" + std::to_string(pFrameInfo->nWidth) +
        "Height_" + std::to_string(pFrameInfo->nHeight) + "_camera.raw";

    saveImageToLocal(szSavePath1.c_str(),pData, pFrameInfo->nFrameLenEx);    

    int nRet = MV_OK;

    MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam;
    memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM_EX));
    int64_t nConvertBufferLenTmp = 0;   //需要格式转换的数据长度
   
    // 判断是否需要HB解码;
    bool bHBFormat = false;
    bHBFormat = IsHBPixelFormat(pFrameInfo->enPixelType);
    if(true == bHBFormat)
    {
        // HB解码
        if(NULL == hb_decode_buffer_)
        {
            hb_decode_buffer_ = new unsigned char[pFrameInfo->nExtendWidth * pFrameInfo->nExtendHeight * 3];
            if(nullptr == hb_decode_buffer_)
            {
                CAMERA_LOG_ERROR("Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
                return -1;
            }
            hb_decode_buffer_len_ = pFrameInfo->nExtendWidth * pFrameInfo->nExtendHeight * 3;
        }

        if(hb_decode_buffer_len_ < pFrameInfo->nExtendWidth * pFrameInfo->nExtendHeight * 3)
        {
            delete[] hb_decode_buffer_;
            hb_decode_buffer_ = new unsigned char[pFrameInfo->nExtendWidth * pFrameInfo->nExtendHeight * 3];
            if(nullptr == hb_decode_buffer_)
            {
                CAMERA_LOG_ERROR( "Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
                return -1;
            }
            hb_decode_buffer_len_ = pFrameInfo->nExtendWidth * pFrameInfo->nExtendHeight * 3;
        }

        MV_CC_HB_DECODE_PARAM stDecodeParam;
        memset(&stDecodeParam, 0, sizeof(MV_CC_HB_DECODE_PARAM));
        stDecodeParam.pSrcBuf = pData;
		stDecodeParam.nSrcLen = pFrameInfo->nFrameLenEx;
        stDecodeParam.pDstBuf = hb_decode_buffer_;
        stDecodeParam.nDstBufSize = hb_decode_buffer_len_;
        
        CAMERA_LOG_DEBUG("before MV_CC_HB_Decode  nSrcLen %d  stDecodeParam.nDstBufSize  %d ", stDecodeParam.nSrcLen, stDecodeParam.nDstBufSize);

        nRet = MV_CC_HB_Decode(handle_, &stDecodeParam);
        if(MV_OK != nRet)
        {
            CAMERA_LOG_ERROR("MV_CC_HB_Decode failed, error: 0x%x", nRet);
            return nRet;
        }
        CAMERA_LOG_DEBUG("Decode success from [0x%x] to [%#x] ", static_cast<unsigned int>(pFrameInfo->enPixelType), static_cast<unsigned int>(stDecodeParam.enDstPixelType));


        //保存图像, 仅存图标识使能后保存
        std::string szSavePath = "./picture/" + std::to_string(pFrameInfo->nFrameNum) +
         "Width_" + std::to_string(stDecodeParam.nWidth) +
         "Height_" + std::to_string(stDecodeParam.nHeight) + "_afterHB.raw";
         saveImageToLocal(szSavePath.c_str(),stDecodeParam.pDstBuf, stDecodeParam.nDstBufLen);


        MvGvspPixelType enDstPixelType = PixelType_Gvsp_Undefined;
        unsigned int nChannelNum = 0;
        // ch:如果是黑白则转换成Mono8 | en:if pixel type is mono, convert it to mono8
        if (IsMonoPixelFormat(stDecodeParam.enDstPixelType))
        {
            nChannelNum = 1;
            enDstPixelType = PixelType_Gvsp_Mono8;
        }
        else
        {
            nChannelNum = 3;
            enDstPixelType = PixelType_Gvsp_RGB8_Packed;
        }

        stConvertParam.nWidth =  stDecodeParam.nWidth;                              //ch:图像宽 | en:image width
        stConvertParam.nHeight = stDecodeParam.nHeight;                            //ch:图像高 | en:image height
        stConvertParam.pSrcData = stDecodeParam.pDstBuf;                             //ch:输入数据缓存 | en:input data buffer
        stConvertParam.nSrcDataLen = stDecodeParam.nDstBufLen;                     //ch:输入数据大小 | en:input data size
        stConvertParam.enSrcPixelType = stDecodeParam.enDstPixelType;                //ch:输入像素格式 | en:input pixel format
        stConvertParam.enDstPixelType = enDstPixelType;                                 //ch:输出像素格式 | en:output pixel format

        nConvertBufferLenTmp = stDecodeParam.nWidth * stDecodeParam.nHeight * nChannelNum;
    }
    else
    {

        MvGvspPixelType enDstPixelType = PixelType_Gvsp_Undefined;
        unsigned int nChannelNum = 0;
        // ch:如果是黑白则转换成Mono8 | en:if pixel type is mono, convert it to mono8
        if (IsMonoPixelFormat(pFrameInfo->enPixelType))
        {
            nChannelNum = 1;
            enDstPixelType = PixelType_Gvsp_Mono8;
        }
        else
        {
            nChannelNum = 3;
            enDstPixelType = PixelType_Gvsp_RGB8_Packed;
        }

        stConvertParam.nWidth = pFrameInfo->nExtendWidth;                 //ch:图像宽 | en:image width
        stConvertParam.nHeight = pFrameInfo->nExtendHeight;               //ch:图像高 | en:image height
        stConvertParam.pSrcData = pData;                         //ch:输入数据缓存 | en:input data buffer
        stConvertParam.nSrcDataLen = pFrameInfo->nFrameLenEx;         //ch:输入数据大小 | en:input data size
        stConvertParam.enSrcPixelType = pFrameInfo->enPixelType;    //ch:输入像素格式 | en:input pixel format
        stConvertParam.enDstPixelType = enDstPixelType;                         //ch:输出像素格式 | en:output pixel format

        nConvertBufferLenTmp = pFrameInfo->nExtendWidth * pFrameInfo->nExtendHeight * nChannelNum;
     }

     //如果图像格式相同，不需要额外转换，直接返回即可
     if(stConvertParam.enSrcPixelType == stConvertParam.enDstPixelType)
     {
        pOut->nWidth = stConvertParam.nWidth;
        pOut->nHeight = stConvertParam.nHeight;
        pOut->enPixelType = stConvertParam.enDstPixelType;
        pOut->pBufAddr = stConvertParam.pSrcData;
        pOut->nBufLen = stConvertParam.nSrcDataLen;

        return nRet;
     }

    if(NULL == convert_buffer_)
    {
        convert_buffer_ = new unsigned char[nConvertBufferLenTmp];
        if(nullptr == convert_buffer_)
        {
            CAMERA_LOG_ERROR("Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
            return -1;
        }
        convert_buffer_len_ = nConvertBufferLenTmp;
    }

    if(convert_buffer_len_ < static_cast<uint64_t>(nConvertBufferLenTmp))
    {
        delete[] convert_buffer_;
        convert_buffer_ = new unsigned char[nConvertBufferLenTmp];
        if(nullptr == convert_buffer_)
        {
            CAMERA_LOG_ERROR("Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
            return -1;
        }
        convert_buffer_len_ = nConvertBufferLenTmp;
    }

    stConvertParam.pDstBuffer = convert_buffer_;                               //ch:输出数据缓存 | en:output data buffer
    stConvertParam.nDstBufferSize = convert_buffer_len_;                       //ch:输出缓存大小 | en:output buffer size

    nRet = MV_CC_ConvertPixelTypeEx(handle_, &stConvertParam);
    if(MV_OK != nRet)
    {
        CAMERA_LOG_ERROR("Failed to convert pixel type, error: 0x%x", nRet);
        return nRet;
    }

    CAMERA_LOG_DEBUG(
        "ConvertPixelType from [%#x] to [%#x] success .",
        static_cast<unsigned int>(stConvertParam.enSrcPixelType),
        static_cast<unsigned int>(stConvertParam.enDstPixelType));


    // 保存转换后的数据 (调试标记使能后, 保存)
    std::string szSavePath = "./picture/" + std::to_string(pFrameInfo->nFrameNum) +
     "Width_" + std::to_string(stConvertParam.nWidth) +
     "Height_" + std::to_string(stConvertParam.nHeight) + "_afterConvert.raw";
     saveImageToLocal(szSavePath.c_str(),convert_buffer_,stConvertParam.nDstLen);

        
    //输出赋值
    pOut->nWidth = stConvertParam.nWidth;
    pOut->nHeight = stConvertParam.nHeight;
    pOut->enPixelType = stConvertParam.enDstPixelType;
    pOut->pBufAddr = convert_buffer_;
    pOut->nBufLen = stConvertParam.nDstLen;

    return nRet;
}



// 图像解码处理
int HikCamera::imageDecodingProcess(MVCC_ROS2_IMAGE_DECODE_INOUT*  pimageinfo)
{
   
    int nRet = MV_OK;

    MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam;
    memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM_EX));
   
    // 判断是否需要HB解码;
    bool bHBFormat = false;
    bHBFormat = IsHBPixelFormat(pimageinfo->enPixelType);
    if(true == bHBFormat)
    {
        uint64_t nNeedLenTmp = pimageinfo->nWidth * pimageinfo->nHeight * 3;

        // HB解码
        if(NULL == hb_decode_buffer_)
        {
            hb_decode_buffer_ = new unsigned char[nNeedLenTmp];
            if(nullptr == hb_decode_buffer_)
            {
                CAMERA_LOG_ERROR( "Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
                return -1;
            }
            hb_decode_buffer_len_ = nNeedLenTmp;
        }

        if(hb_decode_buffer_len_ < nNeedLenTmp)
        {
            delete[] hb_decode_buffer_;
            hb_decode_buffer_ = new unsigned char[nNeedLenTmp * 3];
            if(nullptr == hb_decode_buffer_)
            {
                CAMERA_LOG_ERROR("Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
                return -1;
            }
            hb_decode_buffer_len_ = nNeedLenTmp * 3;
        }

        MV_CC_HB_DECODE_PARAM stDecodeParam;
        memset(&stDecodeParam, 0, sizeof(MV_CC_HB_DECODE_PARAM));

        stDecodeParam.pSrcBuf = pimageinfo->pBufAddr;
		stDecodeParam.nSrcLen = pimageinfo->nBufLen;
        stDecodeParam.pDstBuf = hb_decode_buffer_;
        stDecodeParam.nDstBufSize = hb_decode_buffer_len_;
        
        nRet = MV_CC_HB_Decode(handle_, &stDecodeParam);
        if(MV_OK != nRet)
        {
            CAMERA_LOG_ERROR("MV_CC_HB_Decode failed, error: 0x%x", nRet);
            return nRet;
        }
        CAMERA_LOG_DEBUG("Decode success from [0x%x] to [%#x] ", static_cast<unsigned int>(pimageinfo->enPixelType), static_cast<unsigned int>(stDecodeParam.enDstPixelType));

        //更新输出图像信息
        pimageinfo->enDestPixelType = stDecodeParam.enDstPixelType;
        pimageinfo->pDestBufAddr = stDecodeParam.pDstBuf;
        pimageinfo->nDestBufLen = stDecodeParam.nDstBufSize;
    }
    else
    {
        // 不需要解码，直接输出
        pimageinfo->enDestPixelType = pimageinfo->enPixelType;
        pimageinfo->pDestBufAddr = pimageinfo->pBufAddr;
        pimageinfo->nDestBufLen = pimageinfo->nBufLen;
    }

    return nRet;
}


// 图像格式转换处理
int HikCamera::imageConvertProcess(MVCC_ROS2_IMAGE_CONVERT_INOUT*  pimageinfo)
{
    int nRet = MV_OK;

     //如果图像格式相同，不需要额外转换，直接返回即可
     if(pimageinfo->enPixelType == pimageinfo->enDestPixelType)
     {
        pimageinfo->pDestBufAddr = pimageinfo->pBufAddr;
        pimageinfo->nDestBufLen = pimageinfo->nBufLen;
        return nRet;
     }


     // 格式不同，需要转换为目标格式
    MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam;
    memset(&stConvertParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM_EX));
    uint64_t nConvertBufferLenTmp = pimageinfo->nWidth * pimageinfo->nHeight * 3;

    if(NULL == convert_buffer_)
    {
        convert_buffer_ = new unsigned char[nConvertBufferLenTmp];
        if(nullptr == convert_buffer_)
        {
            CAMERA_LOG_ERROR("Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
            return -1;
        }
        convert_buffer_len_ = nConvertBufferLenTmp;
    }

    if(convert_buffer_len_ < nConvertBufferLenTmp)
    {
        delete[] convert_buffer_;
        convert_buffer_ = new unsigned char[nConvertBufferLenTmp];
        if(nullptr == convert_buffer_)
        {
            CAMERA_LOG_ERROR("Failed to allocate memory for HB decode buffer, error: 0x%x", nRet);
            return -1;
        }
        convert_buffer_len_ = nConvertBufferLenTmp;
    }

  
    stConvertParam.nWidth = pimageinfo->nWidth;                  
    stConvertParam.nHeight = pimageinfo->nHeight;             
    stConvertParam.pSrcData = pimageinfo->pBufAddr;                    
    stConvertParam.nSrcDataLen = pimageinfo->nBufLen;           
    stConvertParam.enSrcPixelType = pimageinfo->enPixelType;    
    stConvertParam.enDstPixelType = pimageinfo->enDestPixelType;                

    stConvertParam.pDstBuffer = convert_buffer_;                     
    stConvertParam.nDstBufferSize = convert_buffer_len_;         

    nRet = MV_CC_ConvertPixelTypeEx(handle_, &stConvertParam);
    if(MV_OK != nRet)
    {
        CAMERA_LOG_ERROR("Failed to convert pixel type, error: 0x%x", nRet);
        return nRet;
    }

    CAMERA_LOG_DEBUG(
        "MV_CC_ConvertPixelTypeEx success from [0x%x] to [%#x].",
        static_cast<unsigned int>(stConvertParam.enSrcPixelType),
        static_cast<unsigned int>(stConvertParam.enDstPixelType));

    //输出赋值
    pimageinfo->pBufAddr = stConvertParam.pDstBuffer ;
    pimageinfo->nBufLen = stConvertParam.nDstLen;

    return nRet;
}



//是否HB格式图像
bool HikCamera::IsHBPixelFormat(MvGvspPixelType enPixelType)
{
    if(enPixelType == PixelType_Gvsp_HB_Mono8 ||
        enPixelType == PixelType_Gvsp_HB_Mono10 ||
        enPixelType == PixelType_Gvsp_HB_Mono10_Packed||
        enPixelType == PixelType_Gvsp_HB_Mono12||
        enPixelType == PixelType_Gvsp_HB_Mono12_Packed||
        enPixelType == PixelType_Gvsp_HB_Mono16||
        enPixelType == PixelType_Gvsp_HB_RGB8_Packed ||
        enPixelType == PixelType_Gvsp_HB_BGR8_Packed ||
        enPixelType == PixelType_Gvsp_HB_RGBA8_Packed ||       
        enPixelType == PixelType_Gvsp_HB_BGRA8_Packed ||
        enPixelType == PixelType_Gvsp_HB_RGB16_Packed ||
        enPixelType == PixelType_Gvsp_HB_BGR16_Packed ||
        enPixelType == PixelType_Gvsp_HB_RGBA16_Packed ||
        enPixelType == PixelType_Gvsp_HB_BGRA16_Packed ||
        enPixelType == PixelType_Gvsp_HB_YUV422_Packed ||
        enPixelType == PixelType_Gvsp_HB_YUV422_YUYV_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerGR8 ||
        enPixelType == PixelType_Gvsp_HB_BayerRG8 ||
        enPixelType == PixelType_Gvsp_HB_BayerGB8 ||
        enPixelType == PixelType_Gvsp_HB_BayerBG8 ||
        enPixelType == PixelType_Gvsp_HB_BayerRBGG8 ||       
        enPixelType == PixelType_Gvsp_HB_BayerGB10 ||
        enPixelType == PixelType_Gvsp_HB_BayerGB10_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerBG10 ||
        enPixelType == PixelType_Gvsp_HB_BayerBG10_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerRG10 ||
        enPixelType == PixelType_Gvsp_HB_BayerRG10_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerGR10 ||
        enPixelType == PixelType_Gvsp_HB_BayerGR10_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerGB12 ||
        enPixelType == PixelType_Gvsp_HB_BayerGB12_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerBG12 ||
        enPixelType == PixelType_Gvsp_HB_BayerBG12_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerRG12 ||
        enPixelType == PixelType_Gvsp_HB_BayerRG12_Packed ||
        enPixelType == PixelType_Gvsp_HB_BayerGR12 ||
        enPixelType == PixelType_Gvsp_HB_BayerGR12_Packed)
    {
        return true;
    }
    else
    {
        return false;
    }       
}

//是否黑白图像
bool HikCamera::IsMonoPixelFormat(MvGvspPixelType enPixelType)
{
    if(enPixelType == PixelType_Gvsp_Mono8 ||
       enPixelType == PixelType_Gvsp_Mono10 ||
       enPixelType == PixelType_Gvsp_Mono10_Packed||
       enPixelType == PixelType_Gvsp_Mono12||
       enPixelType == PixelType_Gvsp_Mono12_Packed||
       enPixelType == PixelType_Gvsp_Mono14||
       enPixelType == PixelType_Gvsp_Mono16)
    {
        return true;
    }
    else
    {
        return false;
    }   
}



// 创建目录的辅助函数
bool HikCamera::createDirectoryRecursively(const char* path) {
    if (path == nullptr || strlen(path) == 0) {
        return false;
    }
    
    std::string pathStr(path);
    
    // 如果是文件路径，提取目录部分
    size_t lastSlash = pathStr.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        pathStr = pathStr.substr(0, lastSlash);
    } else {
        // 没有路径分隔符，说明是当前目录，直接返回成功
        return true;
    }
    
    // 逐级创建目录
    std::string currentPath;
    size_t pos = 0;
    
    // 处理Windows盘符
    if (pathStr.length() >= 2 && pathStr[1] == ':') {
        currentPath = pathStr.substr(0, 2);
        pos = 2;
        if (pathStr.length() > 2 && (pathStr[2] == '/' || pathStr[2] == '\\')) {
            currentPath += pathStr[2];
            pos = 3;
        }
    }
    
    while (pos <= pathStr.length()) {
        // 查找下一个路径分隔符
        size_t nextPos = pathStr.find_first_of("/\\", pos);
        if (nextPos == std::string::npos) {
            nextPos = pathStr.length();
        }
        
        if (nextPos > pos) {
            currentPath += pathStr.substr(pos, nextPos - pos);
            
            // 检查目录是否存在
            if (access(currentPath.c_str(), 0) != 0) {
                // 目录不存在，尝试创建
#ifdef _WIN32
                int result = _mkdir(currentPath.c_str());
#else
                int result = mkdir(currentPath.c_str(), 0755);
#endif
                if (result != 0) {
                    CAMERA_LOG_ERROR("Failed to create directory [%s]", currentPath.c_str());
                    return false;
                }
                CAMERA_LOG_INFO("Created directory [%s]", currentPath.c_str());
            }
            
            // 添加路径分隔符
            if (nextPos < pathStr.length()) {
                currentPath += pathStr[nextPos];
            }
        }
        
        pos = nextPos + 1;
    }
    
    return true;
}

void HikCamera::saveImageToLocal(const char *pFileName, unsigned char * pData, uint64_t nDataLen)
{
    if(false == save_image_)
    {
        return ;
    }

    if(NULL == pData || nDataLen <= 0)
    {
        CAMERA_LOG_ERROR("Invalid parameter: pData is null or nDataLen <= 0");
        return;
    }

    // 创建目录（如果不存在）
    if (!createDirectoryRecursively(pFileName)) {
        CAMERA_LOG_ERROR("Failed to create directory for file [%s]", pFileName);
        return;
    }

    //存图
    FILE* pFileEx = fopen(pFileName, "wb");
    if (NULL == pFileEx)
    {
        CAMERA_LOG_ERROR("Failed to open file [%s]", pFileName);
        return ;
    }
    fwrite(pData, 1, nDataLen, pFileEx);
    fclose(pFileEx);
    pFileEx = NULL;

    return ;
}
