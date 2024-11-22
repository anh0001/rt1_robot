// rt1_hardware/src/odometry_calculator.cpp
#include "rt1_hardware/odometry_calculator.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

void OdometryCalculator::declareParameters()
{
    // Declare parameters with defaults
    rcl_interfaces::msg::ParameterDescriptor rate_desc;
    rate_desc.description = "Publisher rate in Hz";
    rate_desc.floating_point_range.resize(1);
    rate_desc.floating_point_range[0].from_value = 1.0;
    rate_desc.floating_point_range[0].to_value = 1000.0;
    
    this->declare_parameter("publish_rate", 100.0, rate_desc);
}

OdometryCalculator::OdometryCalculator()
: Node("odometry_calculator")
{
    // Initialize parameters
    declareParameters();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    
    // Initialize message timestamps
    last_accel_time_ = this->now();
    last_velocity_time_ = this->now();
    
    // Initialize subscribers with larger queue sizes for 100Hz operation
    accel_sub_ = create_subscription<geometry_msgs::msg::Accel>(
        "rt1/accel", 30, 
        std::bind(&OdometryCalculator::accelCallback, this, std::placeholders::_1));
        
    wrench_sub_ = create_subscription<geometry_msgs::msg::Wrench>(
        "rt1/wrench", 30,
        std::bind(&OdometryCalculator::wrenchCallback, this, std::placeholders::_1));
        
    velocity_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "rt1/velocity", 30,
        std::bind(&OdometryCalculator::velocityCallback, this, std::placeholders::_1));

    // Initialize publisher with larger queue size
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("rt1/odom", 30);
    
    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // Create timer for publishing odometry (100Hz = 10ms)
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&OdometryCalculator::timerCallback, this));
        
    RCLCPP_INFO(this->get_logger(), "Odometry calculator initialized at %.1f Hz", publish_rate_);
    
    last_time_ = this->now().seconds();
}

void OdometryCalculator::accelCallback(const geometry_msgs::msg::Accel::SharedPtr msg)
{
    ax_ = msg->linear.x;
    ay_ = msg->linear.y;
    last_accel_time_ = this->now();
}

void OdometryCalculator::wrenchCallback(const geometry_msgs::msg::Wrench::SharedPtr msg)
{
    // Use wrench data if needed for additional calculations
}

void OdometryCalculator::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    vtheta_ = msg->angular.z;
    last_velocity_time_ = this->now();
}

void OdometryCalculator::updateOdometry()
{
    double current_time = this->now().seconds();
    double dt = current_time - last_time_;
    
    // Check for reasonable dt and data freshness
    if (dt > 0.0 && dt < 0.1) {  // Ensure dt is positive and less than 100ms
        // Check data freshness (within last 100ms)
        auto now = this->now();
        bool fresh_accel = (now - last_accel_time_).seconds() < 0.1;
        bool fresh_velocity = (now - last_velocity_time_).seconds() < 0.1;
        
        if (!fresh_accel || !fresh_velocity) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), 
                                *this->get_clock(),
                                1000, // Warn every 1 second
                                "Stale sensor data detected!");
        }
        
        // Update position using velocity and acceleration
        double dx = vx_ * dt + 0.5 * ax_ * dt * dt;
        double dy = vy_ * dt + 0.5 * ay_ * dt * dt;
        double dtheta = vtheta_ * dt;
        
        // Update velocities using acceleration
        vx_ += ax_ * dt;
        vy_ += ay_ * dt;
        
        // Update pose
        x_ += dx * cos(theta_) - dy * sin(theta_);
        y_ += dx * sin(theta_) + dy * cos(theta_);
        theta_ += dtheta;
        
        // Normalize theta
        while (theta_ > M_PI) theta_ -= 2 * M_PI;
        while (theta_ < -M_PI) theta_ += 2 * M_PI;
    } else if (dt > 0.1) {
        RCLCPP_WARN(this->get_logger(), "Large time delta detected: %.3f seconds", dt);
    }
    
    last_time_ = current_time;
}

void OdometryCalculator::publishOdometry()
{
    // Create and publish odometry message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // Set position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Set orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // Set velocities
    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.linear.y = vy_;
    odom_msg.twist.twist.angular.z = vtheta_;
    
    // Publish odometry message
    odom_pub_->publish(odom_msg);
    
    // Broadcast transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom_msg.header;
    transform.child_frame_id = odom_msg.child_frame_id;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg.pose.pose.orientation;
    
    tf_broadcaster_->sendTransform(transform);
}

void OdometryCalculator::timerCallback()
{
    updateOdometry();
    publishOdometry();
}