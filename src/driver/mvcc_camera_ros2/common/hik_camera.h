#ifndef MVCC_2D_CAMERA_H
#define MVCC_2D_CAMERA_H

// 标准库头文件
#include <string>
#include <memory>
#include <atomic>
#include <mutex>


// 海康相机SDK头文件
#include "PixelType.h"
#include "MvErrorDefine.h"
#include "MvCameraControl.h"

using namespace std;



// 回调函数
#ifdef _WIN32
typedef void(__stdcall *MvImageCallback_ros2)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* frame_info, 
                                                      void* pUser);
#else
typedef void(*MvImageCallback_ros2)(unsigned char * pData, MV_FRAME_OUT_INFO_EX* frame_info, 
                                            void* pUser);
#endif



typedef enum {
    MVCC_ROS2_NODE_INT = 0,         // 整型
    MVCC_ROS2_NODE_FLOAT = 1,       // 浮点型 
    MVCC_ROS2_NODE_BOOL = 2,        // 布尔型
    MVCC_ROS2_NODE_ENUM = 3,        // 枚举值
    MVCC_ROS2_NODE_COMMAND = 4,     // Command类型
    MVCC_ROS2_NODE_STRING = 5       // 字符串值
} MVCC_ROS2_NODE_Type;

typedef union {
    char szStringValue[256];       // 字符串值
    long long intValue;            // 整型值
    float floatValue;              // 浮点值
    bool boolValue;                // 布尔值

    unsigned int        nReserved[16];  // 保留字段
} MVCC_ROS2_NODE_VALUE;


typedef struct _MVCC_ROS2_NODE_Param_ {

    MVCC_ROS2_NODE_Type type;     // 参数类型
    MVCC_ROS2_NODE_VALUE value;   // 参数值

    unsigned int   nReserved[16];  // 保留字段
} MVCC_ROS2_NODE_Param;



typedef struct _MVCC_ROS2_IMAGE_PROC_OUT_
{
    unsigned int            nWidth;                               ///< 图像宽             
    unsigned int            nHeight;                              ///< 图像高          
    enum MvGvspPixelType    enPixelType;                          ///< 像素格式          

    unsigned char*          pBufAddr;                             ///<图像指针地址        
    unsigned int            nBufLen;                              ///< 图像数据长度       

    unsigned int            nRes[16];                              ///<  预留      

}MVCC_ROS2_IMAGE_PROC_OUT;


// HB解码
typedef struct _MVCC_ROS2_IMAGE_DECODE_INOUT_
{
    unsigned int            nWidth;                               ///< [IN] 图像宽             
    unsigned int            nHeight;                              ///< [IN] 图像高          
    enum MvGvspPixelType    enPixelType;                          ///< [IN] 像素格式          

    unsigned char*          pBufAddr;                             ///< [IN] 图像指针地址        
    unsigned int            nBufLen;                              ///< [IN] 图像数据长度       

    enum MvGvspPixelType    enDestPixelType;                     ///< [OUT] 像素格式          
    unsigned char*          pDestBufAddr;                        ///< [OUT] 图像指针地址        
    unsigned int            nDestBufLen;                          ///< [OUT] 图像数据长度       

    unsigned int            nRes[16];                              ///<  预留      

}MVCC_ROS2_IMAGE_DECODE_INOUT;



//格式转换 
typedef struct _MVCC_ROS2_IMAGE_CONVERT_INOUT_
{
    unsigned int            nWidth;                               ///< [IN] 图像宽             
    unsigned int            nHeight;                              ///< [IN] 图像高          
    enum MvGvspPixelType    enPixelType;                          ///< [IN] 像素格式          

    unsigned char*          pBufAddr;                             ///< [IN] 图像指针地址        
    unsigned int            nBufLen;                              ///< [IN] 图像数据长度     
    enum MvGvspPixelType    enDestPixelType;                      ///< [IN] 转换后的像素格式       
         

    unsigned char*          pDestBufAddr;                         ///< [OUT] 图像指针地址        
    unsigned int            nDestBufLen;                          ///< [OUT] 图像数据长度       

    unsigned int            nRes[16];                              ///<  预留      

}MVCC_ROS2_IMAGE_CONVERT_INOUT;



/**
 * @brief 海康相机ROS2封装类
 * 
 * 该类提供了海康相机的完整ROS2接口，包括：
 * - 相机连接和断开
 * - 图像采集和处理
 * - 参数配置和读取
 * - 回调和主动取流两种模式
 */
class HikCamera
{
public:
    /**
     * @brief 构造函数
     */
    HikCamera();
    
    /**
     * @brief 析构函数
     */
    ~HikCamera();

    // 相机生命周期管理
    /**
     * @brief 初始化函数，显示SDK版本，并枚举设备
     * @return 成功返回MV_OK，失败返回错误码
     */
    int initialize();
    
    /**
     * @brief 连接设备
     * @param chSerialNumber 相机序列号，nullptr表示连接第一个设备
     * @return 成功返回MV_OK，失败返回错误码
     */
    int open(const char* chSerialNumber = nullptr);

    /**
     * @brief 开始取流
     * @return 成功返回MV_OK，失败返回错误码
     */
    int start();

    /**
     * @brief 停止取流
     */
    void stop();

    /**
     * @brief 关闭设备
     */
    void close();

    // 回调方式取流
    /**
     * @brief 注册经过处理后的图像回调（相机输出的图像经过解码，格式转换处理后上报给上层应用）
     * @param cbImageCallback 回调函数指针
     * @param pUser 用户数据指针
     * @return 成功返回MV_OK，失败返回错误码
     */
    int registerProcessedImageCallback(MvImageCallback_ros2 cbImageCallback, void* pUser);
    
    /**
     * @brief 注册原始图像回调（直接上报相机输出的图像到应用层）
     * @param cbImageCallback 回调函数指针
     * @param pUser 用户数据指针
     * @return 成功返回MV_OK，失败返回错误码
     */
    int registerImageCallback(MvImageCallback_ros2 cbImageCallback, void* pUser);

    // 主动取流方式
    /**
     * @brief 获取相机图像（建议：创建图像采集线程，使用独立线程循环取图；不建议：使用定时器循环调用）
     * @param pstFrame 输出的帧信息
     * @param nMsec 超时时间（毫秒）
     * @return 成功返回MV_OK，失败返回错误码 ；搭配freeImageBuffer使用
     */
    int getImageBuffer(MV_FRAME_OUT* pstFrame, unsigned int nMsec);
    
    /**
     * @brief 释放图像缓冲区
     * @param pstFrame 要释放的帧信息
     * @return 成功返回MV_OK，失败返回错误码；搭配getImageBuffer使用
     */
    int freeImageBuffer(MV_FRAME_OUT* pstFrame);

    // 图像处理接口
    /**
     * @brief 统一图像处理（进行HB解码，格式转换，统一输出格式）
     * @param pData 输入图像数据
     * @param pFrameInfo 输入帧信息
     * @param pOut 输出处理结果
     * @return 成功返回MV_OK，失败返回错误码
     */
    int imageDecodeAndConvertProcess(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, MVCC_ROS2_IMAGE_PROC_OUT* pOut);
  
    /**
     * @brief 图像解码处理
     * @param pimageinfo 输入输出图像信息
     * @return 成功返回MV_OK，失败返回错误码
     */
    int imageDecodingProcess(MVCC_ROS2_IMAGE_DECODE_INOUT* pimageinfo);

    /**
     * @brief 图像格式转换处理
     * @param pimageinfo 输入输出图像信息
     * @return 成功返回MV_OK，失败返回错误码
     */
    int imageConvertProcess(MVCC_ROS2_IMAGE_CONVERT_INOUT* pimageinfo);

    // 像素格式判断
    /**
     * @brief 判断是否为黑白图像
     * @param enPixelType 像素格式
     * @return 是黑白图像返回true，否则返回false
     */
    bool IsMonoPixelFormat(MvGvspPixelType enPixelType);
    
    /**
     * @brief 判断是否为HB格式图像
     * @param enPixelType 像素格式
     * @return 是HB格式返回true，否则返回false
     */
    bool IsHBPixelFormat(MvGvspPixelType enPixelType);

    // 相机参数操作 - 通用接口
    /**
     * @brief 设置相机参数（通用接口，内部按照类型分别处理）
     * @param chNode 参数节点名称
     * @param pstNodeParam 参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int setCameraParameter(const char* chNode, MVCC_ROS2_NODE_Param* pstNodeParam);
    
    /**
     * @brief 获取相机参数（通用接口，内部按照类型分别处理）
     * @param chNode 参数节点名称
     * @param pstNodeParam 输出的参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int getCameraParameter(const char* chNode, MVCC_ROS2_NODE_Param* pstNodeParam);

    /**
     * @brief 通过字符串设置相机参数
     * @param param_name 参数名称
     * @param value 参数值（字符串形式）
     * @return 成功返回MV_OK，失败返回错误码
     */
    int setCameraParameterbyString(const std::string& param_name, const std::string& value);

    // 相机参数操作 - 按类型操作
    /**
     * @brief 获取整型参数值
     * @param strKey 参数键名
     * @param pIntValue 输出的参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int GetIntValue(const char* strKey, MVCC_INTVALUE_EX *pIntValue);
    
    /**
     * @brief 设置整型参数值
     * @param strKey 参数键名
     * @param nValue 参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int SetIntValue(const char* strKey, int64_t nValue);

    /**
     * @brief 获取枚举型参数值
     * @param strKey 参数键名
     * @param pEnumValue 输出的参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int GetEnumValue(const char* strKey, MVCC_ENUMVALUE *pEnumValue);
    
    /**
     * @brief 设置枚举型参数值
     * @param strKey 参数键名
     * @param nValue 参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int SetEnumValue(const char* strKey, unsigned int nValue);
    
    /**
     * @brief 通过字符串设置枚举型参数值
     * @param strKey 参数键名
     * @param sValue 参数值（字符串形式）
     * @return 成功返回MV_OK，失败返回错误码
     */
    int SetEnumValueByString(const char* strKey, const char* sValue);
    
    /**
     * @brief 获取枚举型参数的符号名称
     * @param strKey 参数键名
     * @param pstEnumEntry 输出的枚举条目
     * @return 成功返回MV_OK，失败返回错误码
     */
    int GetEnumEntrySymbolic(const char* strKey, MVCC_ENUMENTRY* pstEnumEntry);

    /**
     * @brief 获取浮点型参数值
     * @param strKey 参数键名
     * @param pFloatValue 输出的参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int GetFloatValue(const char* strKey, MVCC_FLOATVALUE *pFloatValue);
    
    /**
     * @brief 设置浮点型参数值
     * @param strKey 参数键名
     * @param fValue 参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int SetFloatValue(const char* strKey, float fValue);

    /**
     * @brief 获取布尔型参数值
     * @param strKey 参数键名
     * @param pbValue 输出的参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int GetBoolValue(const char* strKey, bool *pbValue);
    
    /**
     * @brief 设置布尔型参数值
     * @param strKey 参数键名
     * @param bValue 参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int SetBoolValue(const char* strKey, bool bValue);

    /**
     * @brief 获取字符串型参数值
     * @param strKey 参数键名
     * @param pStringValue 输出的参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int GetStringValue(const char* strKey, MVCC_STRINGVALUE *pStringValue);
    
    /**
     * @brief 设置字符串型参数值
     * @param strKey 参数键名
     * @param strValue 参数值
     * @return 成功返回MV_OK，失败返回错误码
     */
    int SetStringValue(const char* strKey, const char * strValue);

    /**
     * @brief 执行一次Command型命令
     * @param strKey 命令键名
     * @return 成功返回MV_OK，失败返回错误码
     */
    int CommandExecute(const char* strKey);

    // 业务专用接口
    /**
     * @brief 设置曝光时间
     * @param fTime 曝光时间（微秒）
     * @return 成功返回MV_OK，失败返回错误码
     */
    int setExposureTime(float fTime);
    
    /**
     * @brief 获取曝光时间
     * @param pfTime 输出的曝光时间（微秒）
     * @return 成功返回MV_OK，失败返回错误码
     */
    int getExposureTime(float* pfTime);

    // 状态查询接口
    /**
     * @brief 检查相机是否已连接
     * @return 已连接返回true，否则返回false
     */
    bool isConnected() const { return handle_ != nullptr; }
    
    /**
     * @brief 获取相机序列号
     * @return 相机序列号字符串
     */
    const std::string& getSerialNumber() const { return serial_number_; }
    
    /**
     * @brief 设置是否保存图像到本地
     * @param enable true保存，false不保存
     */
    void setSaveImageEnabled(bool enable) { save_image_ = enable; }

private:
    // 内部辅助函数
    /**
     * @brief 设备信息处理
     * @param pstMVDevInfo 设备信息
     * @param nOperation 操作类型
     */
    void DeviceInfoProc(MV_CC_DEVICE_INFO* pstMVDevInfo, unsigned int nOperation);
    
    /**
     * @brief 查找设备序列号对应的索引
     * @param pSerial 设备序列号
     * @param pstMVDevInfoList 设备列表
     * @param pnIndex 输出的设备索引
     * @return 找到返回true，否则返回false
     */
    bool findDevIndexBySerial(const char *pSerial, MV_CC_DEVICE_INFO_LIST* pstMVDevInfoList, unsigned int* pnIndex);

    /**
     * @brief 确保缓冲区大小足够
     * @param buffer 缓冲区指针
     * @param current_size 当前缓冲区大小
     * @param required_size 需要的缓冲区大小
     * @return 成功返回true，失败返回false
     */
    bool ensureBufferSize(unsigned char** buffer, int64_t& current_size, int64_t required_size);

    /**
     * @brief 字符串转布尔值
     * @param str 输入字符串
     * @return 对应的布尔值
     */
    bool stringToBool(const std::string& str);

    // 回调函数
#ifdef _WIN32
    static void __stdcall Inner_ImageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
#else
    static void Inner_ImageCallback(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser);
#endif
    
    /**
     * @brief 内部图像回调处理函数
     * @param pData 图像数据
     * @param pFrameInfo 帧信息
     */
    void InnerImageCallbackProc(unsigned char * pData, MV_FRAME_OUT_INFO_EX* pFrameInfo);

    /**
     * @brief 保存图像到本地文件
     * @param pFileName 文件名
     * @param pData 图像数据
     * @param nDataLen 数据长度
     */
    void saveImageToLocal(const char *pFileName, unsigned char * pData, uint64_t nDataLen);

    /**
     * @brief 递归创建目录
     * @param path 文件路径（包含文件名）
     * @return 成功返回true，失败返回false
     */
    bool createDirectoryRecursively(const char* path);


private:
    // 相机句柄和基本信息
    void* handle_ = nullptr;                                    ///< 相机句柄
    std::string serial_number_;                                 ///< 设备序列号

    // 回调相关
    MvImageCallback_ros2 extended_image_callback_ = nullptr;    ///< 扩展图像回调函数
    void* extended_user_ = nullptr;                             ///< 扩展回调用户数据
    std::mutex callback_mutex_;                                 ///< 回调函数互斥锁

    // 图像处理缓冲区
    unsigned char* hb_decode_buffer_ = nullptr;                 ///< HB解码缓存
    uint64_t hb_decode_buffer_len_ = 0;                          ///< HB解码缓存长度
    std::mutex hb_buffer_mutex_;                                ///< HB缓冲区互斥锁

    unsigned char* convert_buffer_ = nullptr;                   ///< 格式转换缓存
    uint64_t convert_buffer_len_ = 0;                            ///< 格式转换缓存长度
    std::mutex convert_buffer_mutex_;                           ///< 转换缓冲区互斥锁

    // 配置选项
    std::atomic<bool> save_image_{false};                       ///< 存图保存标记
    static const int DEFAULT_QUEUE_SIZE = 20;                   ///< 默认图像队列大小
};
#endif
