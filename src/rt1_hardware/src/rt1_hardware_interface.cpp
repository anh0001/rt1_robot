#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosrt_rt1/msg/rt1_sensor.hpp"

class RT1HardwareInterface : public rclcpp::Node
{
public:
    RT1HardwareInterface() : Node("rt1_hardware_interface")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rt1/cmd_vel", 10);
        sensor_sub_ = this->create_subscription<rosrt_rt1::msg::Rt1Sensor>(
            "/rosrt_rt1", 10, std::bind(&RT1HardwareInterface::sensorCallback, this, std::placeholders::_1));
    }

private:
    void sensorCallback(const rosrt_rt1::msg::Rt1Sensor::SharedPtr msg)
    {
        // Process sensor data
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<rosrt_rt1::msg::Rt1Sensor>::SharedPtr sensor_sub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RT1HardwareInterface>());
    rclcpp::shutdown();
    return 0;
}