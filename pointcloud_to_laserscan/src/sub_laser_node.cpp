#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * @brief  This example creates a subclass of Node and uses std::bind() to register a
 *         member function as a callback from the timer.
 * 
 */
class PubSubNode : public rclcpp::Node
{

public:

  PubSubNode() : Node("sub_laser_node") 
  {
    RCLCPP_INFO(this->get_logger(), "Subscribing to topic '/scanner/scan'");
    //  Create BEST EFFORT QoS profile
    auto custom_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    this->sub_= this->create_subscription<sensor_msgs::msg::LaserScan>("/scanner/scan", custom_qos_profile, std::bind(&PubSubNode::laser_callback, this, std::placeholders::_1));
    this->pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/robot/test_topic", 10);
    }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
  {  
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", scan->ranges[0]);
      pub_->publish(*scan);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubSubNode>());
  rclcpp::shutdown();
  return 0;
}