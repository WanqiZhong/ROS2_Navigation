#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "nav_interfaces/msg/wheel_speed.hpp"
#include <serial/serial.h>

class DiffWheelSpeedBridge : public rclcpp::Node
{
    public:
        DiffWheelSpeedBridge(std::string serial_port = "/dev/ttyACM0", int baud_rate = 115200) : Node("diff_wheel_speed_bridge"), ser(serial_port, baud_rate)
        {
            // subscription_ = this->create_subscription<nav_interfaces::msg::WheelSpeed>("/serial/wheel_speed_in", 10, std::bind(&DiffWheelSpeedBridge::sub_topic_callback, this, std::placeholders::_1));
            publisher_ = this->create_publisher<nav_interfaces::msg::WheelSpeed>("/serial/wheel_speed_out", 10);
            publisher_in_ = this->create_publisher<nav_interfaces::msg::WheelSpeed>("/serial/wheel_speed_in", 10);
            // timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&DiffWheelSpeedBridge::pub_timer_callback, this));
        }

        ~DiffWheelSpeedBridge()
        {
            ser.close();
        }

        nav_interfaces::msg::WheelSpeed wheel_speed_out = nav_interfaces::msg::WheelSpeed();
        // nav_interfaces::msg::WheelSpeed wheel_speed_in = nav_interfaces::msg::WheelSpeed();

        void read_encoder_values(int &val_1, int &val_2)
        {
            while (true)
            {
                uint8_t tdata;
                ser.read(&tdata, 1);
                // printf("startByte: %x\n", tdata);
                if (tdata == 0xAA) {
                    uint8_t data[24];
                    ser.read(data, 24);
                    if (data[0] == 0xFF && data[23] == 0xAA) {
                        // uint8_t byte1 = data[0];
                        uint8_t byte2 = data[1];
                        uint8_t byte3 = data[2];
                        uint8_t byte4 = data[3];
                        uint8_t byte5 = data[4];
                        // uint8_t byte6 = data[5];
                        // uint8_t byte7 = data[6];
                        // uint8_t byte8 = data[7];
                        // uint8_t byte9 = data[8];
                        // uint8_t byte10 = data[9];
                        // uint8_t byte11 = data[10];
                        // uint8_t byte12 = data[11];
                        // uint8_t byte13 = data[12];
                        // uint8_t byte14 = data[13];
                        // uint8_t byte15 = data[14];
                        // uint8_t byte16 = data[15];
                        // uint8_t byte17 = data[16];
                        uint8_t byte18 = data[17];
                        uint8_t byte19 = data[18]; 
                        uint8_t byte20 = data[19]; 
                        uint8_t byte21 = data[20]; 
                        uint8_t byte22 = data[21]; 
                        uint8_t byte23 = data[22]; 
                        uint8_t byte24 = data[23]; 

                        int speed1 = (byte2 << 8) | byte3;
                        int speed2 = (byte4 << 8) | byte5;
                        
                        int return_speed1 = (byte18<<8)| byte19;
                        int return_speed2 = (byte20<<8)| byte21;
                        int return_neg_flag1 = byte22;
                        int return_neg_flag2 = byte23;
                        // int joystick_x = (byte6 << 8) | byte7;
                        // int joystick_y = (byte8 << 8) | byte9;
                        // int sensor_1 = (byte10 << 8) | byte11;
                        // int sensor_2 = (byte12 << 8) | byte13;
                        // int sensor_3 = (byte14 << 8) | byte15;
                        // int sensor_4 = (byte16 << 8) | byte17;

                        // if (2100 > joystick_x && joystick_x > 1900) {
                        //     joystick_x = 2000;
                        // }
                        // if (2100 > joystick_y && joystick_y > 1900) {
                        //     joystick_y = 2000;
                        // }
                        if (speed1 > 10000) {
                            speed1 = (65535 - speed1);
                        } else if(speed1 > 0){
                            speed1 = -speed1;
                        }    
                        
                        if (speed2 > 10000) {
                            speed2 = -(65535 - speed2);
                        }

                        val_1 = speed1;
                        val_2 = speed2;


                        if (abs(val_1) > 700 || abs(val_2) > 700){
                            break;
                        }


                        RCLCPP_INFO(this->get_logger(), "Node Received: from encoder '%d' '%d', from stm32 '%d' '%d' '%d' '%d'", val_1, val_2, return_speed1, return_speed2, return_neg_flag1, return_neg_flag2);
                        
                        auto message = nav_interfaces::msg::WheelSpeed();
                        message.speed_l = speed1;
                        message.speed_r = speed2;
                        publisher_in_->publish(message);
                        break;
                    }
                }
            }
               
        }

        void set_motor_values(int val_1, int val_2)
        {
            auto message = nav_interfaces::msg::WheelSpeed();
            message.speed_l = val_1;
            message.speed_r = val_2;
            publisher_->publish(message);
            // RCLCPP_INFO(this->get_logger(), "Node Publishing: '%f' '%f'", message.speed_l, message.speed_r);
        }


    private:
        void pub_timer_callback()
        {
            auto message = this->wheel_speed_out;
            // RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%f'", message.speed_l, message.speed_r);
            publisher_->publish(message);
        }

        // void sub_topic_callback(nav_interfaces::msg::WheelSpeed::SharedPtr msg) 
        // {
        //     this->wheel_speed_in.speed_l = msg->speed_l;
        //     this->wheel_speed_in.speed_r = msg->speed_r;
        //     // RCLCPP_INFO(this->get_logger(), "Received: '%f' '%f'", msg->speed_l, msg->speed_r);
        // }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<nav_interfaces::msg::WheelSpeed>::SharedPtr publisher_;
        rclcpp::Publisher<nav_interfaces::msg::WheelSpeed>::SharedPtr publisher_in_;
        // rclcpp::Subscription<nav_interfaces::msg::WheelSpeed>::SharedPtr subscription_;
        serial::Serial ser;

};