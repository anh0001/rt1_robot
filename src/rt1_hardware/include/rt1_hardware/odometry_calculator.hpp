#ifndef RT1_ODOMETRY_CALCULATOR_HPP
#define RT1_ODOMETRY_CALCULATOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "rosrt_rt1/msg/rt1_sensor.hpp"  // Add the custom message header

class OdometryCalculator : public rclcpp::Node {
public:
    OdometryCalculator();

private:
    // Parameters
    double publish_rate_;  // Hz
    
    // Single subscriber for combined sensor data
    rclcpp::Subscription<rosrt_rt1::msg::Rt1Sensor>::SharedPtr sensor_sub_;
    
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
    rclcpp::Time last_sensor_time_;  // Single timestamp for all sensor data
    
    // Single callback for combined sensor data
    void sensorCallback(const rosrt_rt1::msg::Rt1Sensor::SharedPtr msg);
    void timerCallback();
    
    // Helper functions
    void updateOdometry();
    void publishOdometry();
    void declareParameters();
};

#endif // RT1_ODOMETRY_CALCULATOR_HPP