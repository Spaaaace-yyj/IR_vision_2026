#ifndef LC_SERTAL_TEST_HPP
#define LC_SERIAL_TEST_HPP

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <iomanip>
#include <iostream>
//USER
#include "serial_process.hpp"

// ROS2
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include "auto_aim_interfaces/msg/cmd.hpp"
#include "auto_aim_interfaces/msg/mcu_feed_back.hpp"
// serial_driver
#include <serial_driver/serial_driver.hpp>

#define VISION_SEND_SIZE 36u

//16位标志位定义结构体
#define CAN_FIRE_BIT 0
#define	TRACING_STATE_BIT 1

//按照通信格式修改接收或发送结构体，保证和下位机通信一致，只有浮点位
typedef struct
{
	float target_yaw;
	float tracing;	//0 or 1
} Vision_Send_s;

typedef struct
{
	float yaw;
	float laser_ranging_L0;
	float laser_ranging_L1;
	float laser_ranging_R0;
	float laser_ranging_R1;
} Vision_Recv_s;

class LcSerialTestNode : public rclcpp::Node
{
public:
    explicit LcSerialTestNode(const rclcpp::NodeOptions & options);

    ~LcSerialTestNode();
private:
    void receiveLoop();

    void DecodeData();

    void SendData();

	void OpenPort();
private:

	void CarControlCallback(const auto_aim_interfaces::msg::Cmd::SharedPtr msg);

	inline void setBit(uint16_t &data, uint8_t n, bool state);

	inline bool getBit(uint16_t &data, uint8_t n);

private:
    std::vector<uint8_t> buffer;

    Vision_Recv_s recv_data;
	Vision_Send_s send_data = {
		.target_yaw = 0.0f,
		.tracing = 0.0f,
	};

private:
	//subscription
	rclcpp::Subscription<auto_aim_interfaces::msg::Cmd>::SharedPtr control_sub_;
	//publisher
	rclcpp::Publisher<auto_aim_interfaces::msg::McuFeedBack>::SharedPtr mcu_msg_pub_;
private:
  	rclcpp::TimerBase::SharedPtr serial_timer_;

    std::unique_ptr<IoContext> owned_ctx_;
    std::string device_name_;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

    std::thread receive_thread_;

};

#endif