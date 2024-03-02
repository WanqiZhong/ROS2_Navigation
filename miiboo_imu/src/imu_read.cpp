#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


using namespace std;
using namespace boost::asio;

#define MAX_PACKET_LEN (58) // length of the data

enum ItemID_t
{
    kItemID = 0x90,              /* user programed ID    size: 1 */
    kItemIPAdress = 0x92,        /* ip address           size: 4 */
    kItemAccRaw = 0xA0,          /* raw acc              size: 3x2 */
    kItemAccRawFiltered = 0xA1,
    kItemAccDynamic = 0xA2,
    kItemGyoRaw = 0xB0,          /* raw gyro             size: 3x2 */
    kItemGyoRawFiltered = 0xB1,
    kItemMagRaw = 0xC0,          /* raw mag              size: 3x2 */
    kItemMagRawFiltered = 0xC1,
    kItemAtdE = 0xD0,            /* eular angle          size:3x2 */
    kItemAtdQ = 0xD1,            /* att q,               size:4x4 */
    kItemTemp = 0xE0,
    kItemPressure = 0xF0,        /* pressure             size:1x4 */
    kItemEnd = 0xFF,
};

uint8_t ID;
int16_t AccRaw[3];
int16_t GyoRaw[3];
int16_t MagRaw[3];
float Eular[3];
float pitch, roll, yaw;
float quat[4];
int32_t Pressure;



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("imu_read_node");

    string com_port = "/dev/imu";
    string imu_frame_id = "imu_link";
    string mag_frame_id = "imu_link";
    string imu_topic = "/demo/imu";
    string mag_topic = "/mag";
    node->get_parameter_or("~com_port", com_port, com_port);
    node->get_parameter_or("~imu_frame_id", imu_frame_id, imu_frame_id);
    node->get_parameter_or("~mag_frame_id", mag_frame_id, mag_frame_id);
    node->get_parameter_or("~imu_topic", imu_topic, imu_topic);
    node->get_parameter_or("~mag_topic", mag_topic, mag_topic);

    auto imu_pub = node->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 1000);
    auto mag_pub = node->create_publisher<sensor_msgs::msg::MagneticField>(mag_topic, 1000);
    // auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    io_service iosev;
    serial_port sp(iosev, com_port);

    sp.set_option(serial_port::baud_rate(460800));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));

    int count = 0;
    rclcpp::Rate loop_rate(10);

    RCLCPP_INFO(node->get_logger(), "Start reading IMU data from %s", com_port.c_str());

    while (rclcpp::ok())
    {
        uint8_t buf_tmp[1];
        uint8_t buf[MAX_PACKET_LEN - 1];
        read(sp, buffer(buf_tmp));
        // RCLCPP_INFO(node->get_logger(), "Start reading IMU data is %d", buf_tmp[0]);

        if (buf_tmp[0] == 0x5A)
        {
            read(sp, buffer(buf));
            // RCLCPP_INFO(node->get_logger(), "Start reading IMU data is %d, %d", buf[0], buf[1]);

            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = node->get_clock()->now();
            imu_msg.header.frame_id = imu_frame_id;

            auto mag_msg = sensor_msgs::msg::MagneticField();
            mag_msg.header.stamp = node->get_clock()->now();
            mag_msg.header.frame_id = mag_frame_id;

            int i = 0;
            if (buf[i] == 0xA5) /* user ID */
            {
                // RCLCPP_INFO(node->get_logger(), "User ID is %d", buf[i]);
                
                i += 5;

                if (buf[i + 0] == kItemID) // user ID
                {
                    ID = buf[i + 1];
                }
                if (buf[i + 2] == kItemAccRaw) // Acc value
                {
                    memcpy(AccRaw, &buf[i + 3], 6);
                    imu_msg.linear_acceleration.x = AccRaw[0] / 1000.0 * 9.7887;
                    imu_msg.linear_acceleration.y = AccRaw[1] / 1000.0 * 9.7887;
                    imu_msg.linear_acceleration.z = AccRaw[2] / 1000.0 * 9.7887;
                }
                if (buf[i + 9] == kItemGyoRaw) // Gyro value
                {
                    memcpy(GyoRaw, &buf[i + 10], 6);
                    imu_msg.angular_velocity.x = GyoRaw[0] * M_PI / 10.0 / 180.0;
                    imu_msg.angular_velocity.y = GyoRaw[1] * M_PI / 10.0 / 180.0;
                    imu_msg.angular_velocity.z = GyoRaw[2] * M_PI / 10.0 / 180.0;
                }
                if (buf[i + 16] == kItemMagRaw) // Mag value
                {
                    memcpy(MagRaw, &buf[i + 17], 6);
                    mag_msg.magnetic_field.x = MagRaw[0] / 1000.0 / 10000.0;
                    mag_msg.magnetic_field.y = MagRaw[1] / 1000.0 / 10000.0;
                    mag_msg.magnetic_field.z = MagRaw[2] / 1000.0 / 10000.0;
                }
                if (buf[i + 23] == kItemAtdE) // atd E
                {
                    Eular[0] = ((float)(int16_t)(buf[i + 24] + (buf[i + 25] << 8))) / 100;
                    Eular[1] = ((float)(int16_t)(buf[i + 26] + (buf[i + 27] << 8))) / 100;
                    Eular[2] = ((float)(int16_t)(buf[i + 28] + (buf[i + 29] << 8))) / 10;
                    pitch = Eular[0];
                    roll = Eular[1];
                    yaw = Eular[2];
                    // RCLCPP_INFO(node->get_logger(), "Eular is pitch: %f,  roll: %f, yaw: %f", Eular[0], Eular[1], Eular[2]);
                    // std::cout << Eular[2] <<std::endl;
                    // 使用函数将欧拉角转换为四元数
                    tf2::Quaternion quat;
                    quat.setRPY(roll * M_PI / 180, pitch * M_PI / 180, yaw * M_PI / 180);
                    
                    geometry_msgs::msg::Quaternion quaternion_msg;
                    quaternion_msg.x = quat.x();
                    quaternion_msg.y = quat.y();
                    quaternion_msg.z = quat.z();
                    quaternion_msg.w = 1.0;
                    imu_msg.orientation = tf2::toMsg(quat);
                    // RCLCPP_INFO(node->get_logger(), "Quat: x: %f, y: %f, z: %f, w: %f", quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w);

                    // imu_msg.linear_acceleration_covariance=boost::array<double, 9>;
                }
                if (buf[i + 30] == kItemAtdQ) // atd Q
                {
                    memcpy(quat, &buf[i + 31], 16);
                    imu_msg.orientation.x = quat[1];
                    imu_msg.orientation.y = quat[2];
                    imu_msg.orientation.z = quat[3];
                    imu_msg.orientation.w = quat[0];
                    // RCLCPP_INFO(node->get_logger(), "Quaternion is %f, %f, %f, %f", quat[0], quat[1], quat[2], quat[3]);
                    // std::cout << quat[1] << std::endl;
                }

                imu_pub->publish(imu_msg);
                mag_pub->publish(mag_msg);

                // RCLCPP_INFO(node->get_logger(), "Publish IMU data is %d", count);

                // // Broadcast TF
                // geometry_msgs::msg::TransformStamped transformStamped;
                // transformStamped.header.stamp = node->now();
                // transformStamped.header.frame_id = "world";
                // transformStamped.child_frame_id = imu_frame_id;
                // transformStamped.transform.translation.x = 0.0;
                // transformStamped.transform.translation.y = 0.0;
                // transformStamped.transform.translation.z = 0.0;
                // transformStamped.transform.rotation = imu_msg.orientation;
                // tf_broadcaster->sendTransform(transformStamped);

                count++;
            }
        }
        // loop_rate.sleep();
    }

    RCLCPP_INFO(node->get_logger(), "Exit reading IMU data from %s", com_port.c_str());

    iosev.run();
    rclcpp::shutdown();
    return 0;
}
