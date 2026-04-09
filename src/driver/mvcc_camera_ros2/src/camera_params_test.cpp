#include <rclcpp/rclcpp.hpp>
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

using namespace std::chrono_literals;

class CameraParamTest : public rclcpp::Node
{
public:
    CameraParamTest() : Node("camera_param_test")
    {
        // 创建客户端
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

        // 添加调试信息
        RCLCPP_INFO(this->get_logger(), "Created clients for services");

        // 等待所有服务端启动
        RCLCPP_INFO(this->get_logger(), "Waiting for all services to be available...");
        
        // 等待所有必需的服务启动
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

        bool all_services_ready = false;
        while (!all_services_ready && rclcpp::ok()) {
            all_services_ready = true;
            for (size_t i = 0; i < clients.size(); ++i) {
                if (!clients[i]->wait_for_service(100ms)) {
                    all_services_ready = false;
                    RCLCPP_INFO(this->get_logger(), "Waiting for service: %s", service_names[i].c_str());
                    break;
                }
            }
            
            if (!all_services_ready) {
                RCLCPP_INFO(this->get_logger(), "Some services not available, waiting 1 second...");
                std::this_thread::sleep_for(1s);
            }
        }

        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for services. Exiting.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "All services are ready!");


        // 测试异步调用
     //   testExposureTimeAsync();

      // 测试同步调用
        test_float_ExposureTime();
        test_string_DeviceUserID();

        test_int_AcquisitionLineRate();
        test_command_FindMe();
        test_enum_UserSetSelector();

        test_bool_ReverseX();


        
        #if 0
        // 直接调用测试函数
        test_float_ExposureTime();
        #endif 


    }

private:

    void testExposureTimeAsync()
    {
        RCLCPP_INFO(this->get_logger(), "=== Testing ExposureTime parameter (Async) ===");

        auto get_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetFloatValue::Request>();
        get_request->str_key = "ExposureTime";
        
        // 使用回调函数处理响应
        auto response_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetFloatValue>::SharedFuture future)
         {
            auto response = future.get();
            if (response->success) 
            {
                RCLCPP_INFO(this->get_logger(), "Current ExposureTime: %f", response->value);
                // 处理成功逻辑
            } 
            else 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get ExposureTime: %s", response->message.c_str());
            }
        };
        
        auto future = get_float_value_client_->async_send_request(get_request, response_callback);
        RCLCPP_INFO(this->get_logger(), "Request sent asynchronously");
    }




    void test_float_ExposureTime()
    {
        RCLCPP_INFO(this->get_logger(), "=== Testing ExposureTime parameter ===");

        auto get_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetFloatValue::Request>();
        get_request->str_key = "ExposureTime";
        
        RCLCPP_INFO(this->get_logger(), "Sending request to get_float_value service");
        
        // 使用异步方式获取当前曝光时间，直接传递回调函数
        auto get_response_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetFloatValue>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Current ExposureTime: %f", response->value);
                
                // 获取成功后设置新值
                auto set_request = std::make_shared<mvcc_camera_ros2_interface::srv::SetFloatValue::Request>();
                set_request->str_key = "ExposureTime";
                set_request->value = response->value + 100.0f;
                
                RCLCPP_INFO(this->get_logger(), "Sending request to set_float_value service");
                
                // 使用回调处理设置响应
                auto set_response_callback = [this, set_request](rclcpp::Client<mvcc_camera_ros2_interface::srv::SetFloatValue>::SharedFuture set_future)
                {
                    auto set_response = set_future.get();
                    if (set_response->success)
                    {
                        RCLCPP_INFO(this->get_logger(), "Successfully set ExposureTime to %f", set_request->value);
                        
                        // 设置成功后确认设置
                        auto get_request2 = std::make_shared<mvcc_camera_ros2_interface::srv::GetFloatValue::Request>();
                        get_request2->str_key = "ExposureTime";
                        
                        RCLCPP_INFO(this->get_logger(), "Sending second request to get_float_value service");
                        
                        // 使用回调处理确认响应
                        auto confirm_response_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetFloatValue>::SharedFuture confirm_future)
                        {
                            auto response2 = confirm_future.get();
                            if (response2->success)
                            {
                                RCLCPP_INFO(this->get_logger(), "Confirmed ExposureTime is now: %f", response2->value);
                            }
                        };
                        
                        get_float_value_client_->async_send_request(get_request2, confirm_response_callback);
                    }
                };
                
                set_float_value_client_->async_send_request(set_request, set_response_callback);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get ExposureTime: %s", response->message.c_str());
            }
        };
        
        get_float_value_client_->async_send_request(get_request, get_response_callback);
    }

     void test_string_DeviceUserID()
     {
        // 1. 设置 string 类型，节点名 "DeviceUserID" 值 "123"
        auto set_string_request = std::make_shared<mvcc_camera_ros2_interface::srv::SetStringValue::Request>();
        set_string_request->str_key = "DeviceUserID";
        set_string_request->value = "123";
        
        auto set_string_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::SetStringValue>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully set DeviceUserID to '123'");
                
                // 设置成功后读取值
                auto get_string_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetStringValue::Request>();
                get_string_request->str_key = "DeviceUserID";
                
                auto get_string_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetStringValue>::SharedFuture future)
                {
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
                
                get_string_value_client_->async_send_request(get_string_request, get_string_callback);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set DeviceUserID: %s", response->message.c_str());
            }
        };
        
        set_string_value_client_->async_send_request(set_string_request, set_string_callback);
    }


    void test_int_AcquisitionLineRate()
     {
        // 设置 int 类型，节点名 "AcquisitionLineRate" 值 505
        auto set_int_request = std::make_shared<mvcc_camera_ros2_interface::srv::SetIntegerValue::Request>();
        set_int_request->str_key = "AcquisitionLineRate";
        set_int_request->value = 505;
        
        auto set_int_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::SetIntegerValue>::SharedFuture future)
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully set AcquisitionLineRate to 505");
                
                // 设置成功后读取值
                auto get_int_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetIntegerValue::Request>();
                get_int_request->str_key = "AcquisitionLineRate";
                
                auto get_int_callback = [this](rclcpp::Client<mvcc_camera_ros2_interface::srv::GetIntegerValue>::SharedFuture future)
                {
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
                
                get_integer_value_client_->async_send_request(get_int_request, get_int_callback);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set AcquisitionLineRate: %s", response->message.c_str());
            }
        };
        
        set_integer_value_client_->async_send_request(set_int_request, set_int_callback);
     }
     void test_command_FindMe()
    {
        //  设置command类型，节点名 "FindMe"
        auto stop_command_request = std::make_shared<mvcc_camera_ros2_interface::srv::ExecuteCommand::Request>();
        stop_command_request->str_key = "FindMe";
        auto stop_command_future = execute_command_client_->async_send_request(stop_command_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), stop_command_future, 5s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = stop_command_future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully executed FindMe command");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to execute FindMe command: %s", response->message.c_str());
            }
        }
     }
    
    void test_enum_UserSetSelector()
    {
        // 设置 enum 类型，节点名 "UserSetSelector" 值 "UserSet1"
        auto set_enum_off_request = std::make_shared<mvcc_camera_ros2_interface::srv::SetEnumValue::Request>();
        set_enum_off_request->str_key = "UserSetSelector";
        set_enum_off_request->strvalue = "UserSet1";
        auto set_enum_off_future = set_enum_value_client_->async_send_request(set_enum_off_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_enum_off_future, 5s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = set_enum_off_future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Successfully set  [%s] to  [%s]",
                set_enum_off_request->str_key.c_str(), set_enum_off_request->strvalue.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set [%s] to [%s] error: [%s]",
                set_enum_off_request->str_key.c_str(), set_enum_off_request->strvalue.c_str(), response->message.c_str());
            }
        }

        // 读取 enum 类型，节点名 "UserSetSelector" 的值，并打印
        auto get_enum_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetEnumValue::Request>();
        get_enum_request->str_key = "UserSetSelector";
        auto get_enum_future = get_enum_value_client_->async_send_request(get_enum_request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_enum_future, 5s) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = get_enum_future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Get  [%s] value: [%ld], string value: [%s]", 
                get_enum_request->str_key.c_str(),  response->value, response->str_value.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get  [%s] value, error[%s]",
                 get_enum_request->str_key.c_str(), response->message.c_str());
            }
        }
        
     }
        

         void test_bool_ReverseX()
        {
            // 设置 bool 类型，节点名 "ReverseX" 值 true
            auto set_bool_request = std::make_shared<mvcc_camera_ros2_interface::srv::SetBoolValue::Request>();
            set_bool_request->str_key = "ReverseX";
            set_bool_request->value = true;
            auto set_bool_future = set_bool_value_client_->async_send_request(set_bool_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), set_bool_future, 5s) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = set_bool_future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "Successfully set ReverseX to true");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set ReverseX: %s", response->message.c_str());
                }
            }

            // 读取 bool 类型，节点名 "ReverseX" 的值，并打印
            auto get_bool_request = std::make_shared<mvcc_camera_ros2_interface::srv::GetBoolValue::Request>();
            get_bool_request->str_key = "ReverseX";
            auto get_bool_future = get_bool_value_client_->async_send_request(get_bool_request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), get_bool_future, 5s) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = get_bool_future.get();
                if (response->success)
                {
                    RCLCPP_INFO(this->get_logger(), "ReverseX value: %s", response->value ? "true" : "false");
                }
                else
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get ReverseX: %s", response->message.c_str());
                }
            }
        }



    rclcpp::TimerBase::SharedPtr timer_;
    
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
};

int main(int argc, char **argv)
{
    // 只在客户端设置 [CycloneDDS 和 Fast DDS 是ROS2支持的两种不同的DDS实现]   Fast DDS通常是ROS2的默认安装，兼容性更好
    setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp", 1);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraParamTest>();
    
    // 使用spin_some循环处理回调，避免节点被重复添加到执行器
    rclcpp::spin_some(node);
    
    rclcpp::shutdown();
    return 0;
}
