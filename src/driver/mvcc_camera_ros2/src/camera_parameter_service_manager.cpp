#include "../common/camera_parameter_service_manager.h"

CameraParameterServiceManager::CameraParameterServiceManager(rclcpp::Node* node, HikCamera* camera)
    : node_(node), camera_(camera)
{
    if (node_ == nullptr) {
        throw std::invalid_argument("Node pointer cannot be null");
    }
    
    if (camera_ == nullptr) {
        throw std::invalid_argument("Camera pointer cannot be null");
    }
    
    RCLCPP_INFO(node_->get_logger(), "Initializing Camera Parameter Service Manager...");
    
    // 初始化所有服务
    initializeServices();
    
    RCLCPP_INFO(node_->get_logger(), "Camera Parameter Service Manager initialized successfully");
}

CameraParameterServiceManager::~CameraParameterServiceManager()
{
    RCLCPP_INFO(node_->get_logger(), "Camera Parameter Service Manager destroyed");
}

void CameraParameterServiceManager::initializeServices()
{
    // 创建所有参数服务
    get_float_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::GetFloatValue>(
        "/get_float_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetFloatValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::GetFloatValue::Response> response) {
            this->handleGetFloatValue(request, response);
        });

    set_float_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::SetFloatValue>(
        "/set_float_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetFloatValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::SetFloatValue::Response> response) {
            this->handleSetFloatValue(request, response);
        });

    get_string_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::GetStringValue>(
        "/get_string_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetStringValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::GetStringValue::Response> response) {
            this->handleGetStringValue(request, response);
        });

    set_string_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::SetStringValue>(
        "/set_string_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetStringValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::SetStringValue::Response> response) {
            this->handleSetStringValue(request, response);
        });

    execute_command_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::ExecuteCommand>(
        "/execute_command",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::ExecuteCommand::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::ExecuteCommand::Response> response) {
            this->handleExecuteCommand(request, response);
        });

    get_enum_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::GetEnumValue>(
        "/get_enum_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetEnumValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::GetEnumValue::Response> response) {
            this->handleGetEnumValue(request, response);
        });

    set_enum_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::SetEnumValue>(
        "/set_enum_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetEnumValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::SetEnumValue::Response> response) {
            this->handleSetEnumValue(request, response);
        });

    get_integer_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::GetIntegerValue>(
        "/get_integer_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetIntegerValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::GetIntegerValue::Response> response) {
            this->handleGetIntegerValue(request, response);
        });

    set_integer_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::SetIntegerValue>(
        "/set_integer_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetIntegerValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::SetIntegerValue::Response> response) {
            this->handleSetIntegerValue(request, response);
        });

    get_bool_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::GetBoolValue>(
        "/get_bool_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetBoolValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::GetBoolValue::Response> response) {
            this->handleGetBoolValue(request, response);
        });

    set_bool_value_service_ = node_->create_service<mvcc_camera_ros2_interface::srv::SetBoolValue>(
        "/set_bool_value",
        [this](const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetBoolValue::Request> request,
               std::shared_ptr<mvcc_camera_ros2_interface::srv::SetBoolValue::Response> response) {
            this->handleSetBoolValue(request, response);
        });
}

// GetFloatValue服务的实现
void CameraParameterServiceManager::handleGetFloatValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetFloatValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::GetFloatValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleGetFloatValue: recv Get [%s] command", request->str_key.c_str());
    
    // 使用通用接口获取浮点型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_FLOAT;
    
    int nRet = camera_->getCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->value = param.value.floatValue;
        response->success = true;
        response->message = "Get float value successfully";
        RCLCPP_INFO(node_->get_logger(), "Get [%s] value: %f", 
                   request->str_key.c_str(), param.value.floatValue);
    } else {
        response->success = false;
        response->message = "Failed to get float value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to get [%s] float value, error: 0x%x", 
                    request->str_key.c_str(), nRet);
    }
}

// SetFloatValue服务的实现
void CameraParameterServiceManager::handleSetFloatValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetFloatValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::SetFloatValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleSetFloatValue: recv Set [%s] to %f command", 
               request->str_key.c_str(), request->value);
    
    // 使用通用接口设置浮点型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_FLOAT;
    param.value.floatValue = request->value;
    
    int nRet = camera_->setCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->success = true;
        response->message = "Set float value successfully";
        RCLCPP_INFO(node_->get_logger(), "Set [%s] to %f successfully", 
                   request->str_key.c_str(), request->value);
    } else {
        response->success = false;
        response->message = "Failed to set float value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to set [%s] to %f, error: 0x%x", 
                    request->str_key.c_str(), request->value, nRet);
    }
}

// GetStringValue服务的实现
void CameraParameterServiceManager::handleGetStringValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetStringValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::GetStringValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleGetStringValue: recv Get [%s] command", request->str_key.c_str());
    
    // 使用通用接口获取字符串型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_STRING;
    
    int nRet = camera_->getCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->value = std::string(param.value.szStringValue);
        response->success = true;
        response->message = "Get string value successfully";
        RCLCPP_INFO(node_->get_logger(), "Get [%s] value: %s", 
                   request->str_key.c_str(), param.value.szStringValue);
    } else {
        response->success = false;
        response->message = "Failed to get string value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to get [%s] string value, error: 0x%x", 
                    request->str_key.c_str(), nRet);
    }
}

// SetStringValue服务的实现
void CameraParameterServiceManager::handleSetStringValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetStringValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::SetStringValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    if (request->value.empty()) {
        response->success = false;
        response->message = "Parameter value is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleSetStringValue: recv Set [%s] to %s command", 
               request->str_key.c_str(), request->value.c_str());
    
    // 使用通用接口设置字符串型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_STRING;
    strncpy(param.value.szStringValue, request->value.c_str(), sizeof(param.value.szStringValue) - 1);
    param.value.szStringValue[sizeof(param.value.szStringValue) - 1] = '\0';
    
    int nRet = camera_->setCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->success = true;
        response->message = "Set string value successfully";
        RCLCPP_INFO(node_->get_logger(), "Set [%s] to %s successfully", 
                   request->str_key.c_str(), request->value.c_str());
    } else {
        response->success = false;
        response->message = "Failed to set string value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to set [%s] to %s, error: 0x%x", 
                    request->str_key.c_str(), request->value.c_str(), nRet);
    }
}

// ExecuteCommand服务的实现
void CameraParameterServiceManager::handleExecuteCommand(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::ExecuteCommand::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::ExecuteCommand::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Command key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleExecuteCommand: recv Execute [%s] command", request->str_key.c_str());
        
    // 使用通用接口设置字符串型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_COMMAND;


    int nRet = camera_->setCameraParameter(request->str_key.c_str(), &param);
    if (MV_OK == nRet) {
        response->success = true;
        response->message = "execute command  successfully";
        RCLCPP_INFO(node_->get_logger(), "execute command [%s] successfully", 
                   request->str_key.c_str());
    } else {
        response->success = false;
        response->message = "execute command, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed execute command [%s], error: 0x%x", 
                    request->str_key.c_str(), nRet);
    }
}

// GetEnumValue服务的实现
void CameraParameterServiceManager::handleGetEnumValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetEnumValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::GetEnumValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleGetEnumValue: recv Get [%s] command", request->str_key.c_str());
    
    // 使用通用接口获取枚举型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_ENUM;
    
    int nRet = camera_->getCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->value = param.value.intValue;  // 枚举值存储在intValue中
        response->str_value = std::string(param.value.szStringValue);  // 字符串表示
        response->success = true;
        response->message = "Get enum value successfully";
        RCLCPP_INFO(node_->get_logger(), "Get [%s] value: %lld, string: %s", 
                   request->str_key.c_str(), param.value.intValue, param.value.szStringValue);
    } else {
        response->success = false;
        response->message = "Failed to get enum value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to get [%s] enum value, error: 0x%x", 
                    request->str_key.c_str(), nRet);
    }
}

// SetEnumValue服务的实现
void CameraParameterServiceManager::handleSetEnumValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetEnumValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::SetEnumValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    if (request->strvalue.empty()) {
        response->success = false;
        response->message = "Parameter value is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleSetEnumValue: recv Set [%s] to %s command", 
               request->str_key.c_str(), request->strvalue.c_str());
    
    // 使用通用接口设置枚举型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_ENUM;
    strncpy(param.value.szStringValue, request->strvalue.c_str(), sizeof(param.value.szStringValue) - 1);
    param.value.szStringValue[sizeof(param.value.szStringValue) - 1] = '\0';
    
    int nRet = camera_->setCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->success = true;
        response->message = "Set enum value successfully";
        RCLCPP_INFO(node_->get_logger(), "Set [%s] to %s successfully", 
                   request->str_key.c_str(), request->strvalue.c_str());
    } else {
        response->success = false;
        response->message = "Failed to set enum value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to set [%s] to %s, error: 0x%x", 
                    request->str_key.c_str(), request->strvalue.c_str(), nRet);
    }
}

// GetIntegerValue服务的实现
void CameraParameterServiceManager::handleGetIntegerValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetIntegerValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::GetIntegerValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleGetIntegerValue: recv Get [%s] command", request->str_key.c_str());
    
    // 使用通用接口获取整型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_INT;
    
    int nRet = camera_->getCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->value = param.value.intValue;
        response->success = true;
        response->message = "Get integer value successfully";
        RCLCPP_INFO(node_->get_logger(), "Get [%s] value: %lld", 
                   request->str_key.c_str(), param.value.intValue);
    } else {
        response->success = false;
        response->message = "Failed to get integer value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to get [%s] integer value, error: 0x%x", 
                    request->str_key.c_str(), nRet);
    }
}

// SetIntegerValue服务的实现
void CameraParameterServiceManager::handleSetIntegerValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetIntegerValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::SetIntegerValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleSetIntegerValue: recv Set [%s] to %lld command", 
               request->str_key.c_str(), (long long)request->value);
    
    // 使用通用接口设置整型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_INT;
    param.value.intValue = request->value;
    
    int nRet = camera_->setCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->success = true;
        response->message = "Set integer value successfully";
        RCLCPP_INFO(node_->get_logger(), "Set [%s] to %lld successfully", 
                   request->str_key.c_str(), (long long)request->value);
    } else {
        response->success = false;
        response->message = "Failed to set integer value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to set [%s] to %lld, error: 0x%x", 
                    request->str_key.c_str(), (long long)request->value, nRet);
    }
}

// GetBoolValue服务的实现
void CameraParameterServiceManager::handleGetBoolValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::GetBoolValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::GetBoolValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleGetBoolValue: recv Get [%s] command", request->str_key.c_str());
    
    // 使用通用接口获取布尔型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_BOOL;
    
    int nRet = camera_->getCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->value = param.value.boolValue;
        response->success = true;
        response->message = "Get bool value successfully";
        RCLCPP_INFO(node_->get_logger(), "Get [%s] value: %s", 
                   request->str_key.c_str(), param.value.boolValue ? "true" : "false");
    } else {
        response->success = false;
        response->message = "Failed to get bool value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to get [%s] bool value, error: 0x%x", 
                    request->str_key.c_str(), nRet);
    }
}

// SetBoolValue服务的实现
void CameraParameterServiceManager::handleSetBoolValue(
    const std::shared_ptr<mvcc_camera_ros2_interface::srv::SetBoolValue::Request> request,
    std::shared_ptr<mvcc_camera_ros2_interface::srv::SetBoolValue::Response> response)
{
    if (request->str_key.empty()) {
        response->success = false;
        response->message = "Parameter key is empty";
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "handleSetBoolValue: recv Set [%s] to %s command", 
               request->str_key.c_str(), request->value ? "true" : "false");
    
    // 使用通用接口设置布尔型参数值
    MVCC_ROS2_NODE_Param param;
    param.type = MVCC_ROS2_NODE_BOOL;
    param.value.boolValue = request->value;
    
    int nRet = camera_->setCameraParameter(request->str_key.c_str(), &param);
    
    if (MV_OK == nRet) {
        response->success = true;
        response->message = "Set bool value successfully";
        RCLCPP_INFO(node_->get_logger(), "Set [%s] to %s successfully", 
                   request->str_key.c_str(), request->value ? "true" : "false");
    } else {
        response->success = false;
        response->message = "Failed to set bool value, error: " + std::to_string(nRet);
        RCLCPP_ERROR(node_->get_logger(), "Failed to set [%s] to %s, error: 0x%x", 
                    request->str_key.c_str(), request->value ? "true" : "false", nRet);
    }
}
