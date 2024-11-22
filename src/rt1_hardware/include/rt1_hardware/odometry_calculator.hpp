#ifndef RT1_ODOMETRY_CALCULATOR_HPP
#define RT1_ODOMETRY_CALCULATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

class OdometryCalculator : public rclcpp::Node {
public:
    OdometryCalculator();

private:
    // Parameters
    double publish_rate_;  // Hz
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Accel>::SharedPtr accel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr wrench_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timer for publishing odometry at fixed rate
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State variables
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double vtheta_ = 0.0;
    double ax_ = 0.0;
    double ay_ = 0.0;
    double last_time_ = 0.0;
    
    // Message timestamp tracking
    rclcpp::Time last_accel_time_;
    rclcpp::Time last_velocity_time_;
    
    // Callbacks
    void accelCallback(const geometry_msgs::msg::Accel::SharedPtr msg);
    void wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg);
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timerCallback();
    
    // Helper functions
    void updateOdometry();
    void publishOdometry();
    void declareParameters();
};

#endif // RT1_ODOMETRY_CALCULATOR_HPP