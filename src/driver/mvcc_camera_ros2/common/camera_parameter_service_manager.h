#ifndef MVCC_CAMERA_PARAMETER_SERVICE_MANAGER_H
#define MVCC_CAMERA_PARAMETER_SERVICE_MANAGER_H

#include "rclcpp/rclcpp.hpp"
#include "../common/hik_camera.h"
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

using namespace std;

/**
 * @brief 相机参数服务管理器类
 * 
 * 该类负责管理所有相机参数相关的ROS2服务，包括：
 * - 浮点型参数的读写服务
 * - 字符串型参数的读写服务
 * - 整型参数的读写服务
 * - 布尔型参数的读写服务
 * - 枚举型参数的读写服务
 * - 命令执行服务
 */
class CameraParameterServiceManager
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针
     * @param camera 海康相机对象指针
     */
    CameraParameterServiceManager(rclcpp::Node* node, HikCamera* camera);
    
    /**
     * @brief 析构函数
     */
    ~CameraParameterServiceManager();

private:
    // 服务回调函数声明
    void handleGetFloatValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetFloatValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::GetFloatValue::Response> response);
    
    void handleSetFloatValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetFloatValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::SetFloatValue::Response> response);

    void handleGetStringValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetStringValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::GetStringValue::Response> response);
    
    void handleSetStringValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetStringValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::SetStringValue::Response> response);

    void handleExecuteCommand(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::ExecuteCommand::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::ExecuteCommand::Response> response);

    void handleGetEnumValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetEnumValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::GetEnumValue::Response> response);

    void handleSetEnumValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetEnumValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::SetEnumValue::Response> response);

    void handleGetIntegerValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetIntegerValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::GetIntegerValue::Response> response);

    void handleSetIntegerValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetIntegerValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::SetIntegerValue::Response> response);

    void handleGetBoolValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetBoolValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::GetBoolValue::Response> response);

    void handleSetBoolValue(
        const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetBoolValue::Request> request,
        std::shared_ptr<mvcc_camera_ros2_interface::srv::SetBoolValue::Response> response);

    /**
     * @brief 初始化所有服务
     */
    void initializeServices();

private:
    rclcpp::Node* node_;                    ///< ROS2节点指针
    HikCamera* camera_;                      ///< 海康相机对象指针
    
    // 服务成员变量
    rclcpp::Service<mvcc_camera_ros2_interface::srv::GetFloatValue>::SharedPtr get_float_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::SetFloatValue>::SharedPtr set_float_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::GetStringValue>::SharedPtr get_string_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::SetStringValue>::SharedPtr set_string_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::ExecuteCommand>::SharedPtr execute_command_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::GetEnumValue>::SharedPtr get_enum_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::SetEnumValue>::SharedPtr set_enum_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::GetIntegerValue>::SharedPtr get_integer_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::SetIntegerValue>::SharedPtr set_integer_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::GetBoolValue>::SharedPtr get_bool_value_service_;
    rclcpp::Service<mvcc_camera_ros2_interface::srv::SetBoolValue>::SharedPtr set_bool_value_service_;
};

#endif // MVCC_CAMERA_PARAMETER_SERVICE_MANAGER_H
