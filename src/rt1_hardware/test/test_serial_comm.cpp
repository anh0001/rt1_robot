#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rosrt_rt1/msg/rt1_sensor.hpp"

class SerialCommTest : public rclcpp::Node
{
public:
    SerialCommTest() : Node("serial_comm_test")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rt1/cmd_vel", 10);
        sensor_sub_ = this->create_subscription<rosrt_rt1::msg::Rt1Sensor>(
            "/rosrt_rt1", 10, std::bind(&SerialCommTest::sensorCallback, this, std::placeholders::_1));
        
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SerialCommTest::timerCallback, this));
    }

private:
    void sensorCallback(const rosrt_rt1::msg::Rt1Sensor::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received sensor data: linear velocity = %f", msg->velocity.linear.x);
    }

    void timerCallback()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.1;  // Move forward at 0.1 m/s
        cmd_vel_pub_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Published velocity command: linear.x = %f", twist_msg.linear.x);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<rosrt_rt1::msg::Rt1Sensor>::SharedPtr sensor_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialCommTest>());
    rclcpp::shutdown();
    return 0;
}