#include <iostream>
#include "boost/circular_buffer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"

class OmniStateToTwist : public rclcpp::Node
{
public:
    OmniStateToTwist() 
        : Node("omni_state_to_twist"), 
          previous_proposed_twists_(consecutive_nonzero_)
    {
        // Récupération des paramètres ROS 2
        movement_scale_ = this->declare_parameter<double>("movement_scale", 0.25);
        deadband_ = this->declare_parameter<double>("deadband", 0.01);
        consecutive_nonzero_ = this->declare_parameter<int>("consecutive_nonzero", 6);

        // Configuration des abonnements et publications
        std::string omni_name = this->declare_parameter<std::string>("omni_name", "default_omni");
        omni_sub_ = this->create_subscription<omni_msgs::msg::OmniState>(
            omni_name + "/state", 
            rclcpp::QoS(QUEUE_LENGTH), 
            std::bind(&OmniStateToTwist::omniCallback, this, std::placeholders::_1));
        
        button_sub_ = this->create_subscription<omni_msgs::msg::OmniButtonEvent>(
            omni_name + "/button", 
            rclcpp::QoS(QUEUE_LENGTH), 
            std::bind(&OmniStateToTwist::buttonCallback, this, std::placeholders::_1));
        
        twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "servo_server/cartesian_command", 
            rclcpp::QoS(QUEUE_LENGTH));
        
        RCLCPP_INFO(this->get_logger(), "OmniStateToTwist node initialized successfully.");
    }

private:
    void buttonCallback(const omni_msgs::msg::OmniButtonEvent::SharedPtr msg) {
        movement_active_ = msg->grey_button;
    }

    void omniCallback(const omni_msgs::msg::OmniState::SharedPtr msg) {
        auto callback_time = this->now();

        if (movement_active_) {
            auto twist = geometry_msgs::msg::TwistStamped();
            twist.header.stamp = callback_time;
            twist.twist.linear = msg->velocity;
            twist.twist.angular = quaternionPosesToAngularVelocity(
                last_orientation_, 
                msg->pose.orientation, 
                (callback_time - last_processed_time_).seconds());

            deadbandFilterNoise(twist, deadband_);
            previousMovementFilterNoise(twist, consecutive_nonzero_);
            scaleTwist(twist, movement_scale_);
            transformTwistToUrFrame(twist);
            twist_pub_->publish(twist);
        }

        last_processed_time_ = callback_time;
        last_orientation_ = msg->pose.orientation;
    }

    geometry_msgs::msg::Vector3 quaternionPosesToAngularVelocity(
        const geometry_msgs::msg::Quaternion& q1, 
        const geometry_msgs::msg::Quaternion& q2, 
        double dt) 
    {
        geometry_msgs::msg::Vector3 angular_velocity;
        angular_velocity.x = (2.0 / dt) * (q1.w * q2.x - q1.x * q2.w - q1.y * q2.z + q1.z * q2.y);
        angular_velocity.y = (2.0 / dt) * (q1.w * q2.y + q1.x * q2.z - q1.y * q2.w - q1.z * q2.x);
        angular_velocity.z = (2.0 / dt) * (q1.w * q2.z - q1.x * q2.y + q1.y * q2.x - q1.z * q2.w);
        return angular_velocity;
    }

    void deadbandFilterNoise(geometry_msgs::msg::TwistStamped& twist, const double& deadband) {
        if (std::abs(twist.twist.linear.x) < deadband) twist.twist.linear.x = 0.0;
        if (std::abs(twist.twist.linear.y) < deadband) twist.twist.linear.y = 0.0;
        if (std::abs(twist.twist.linear.z) < deadband) twist.twist.linear.z = 0.0;
        if (std::abs(twist.twist.angular.x) < deadband) twist.twist.angular.x = 0.0;
        if (std::abs(twist.twist.angular.y) < deadband) twist.twist.angular.y = 0.0;
        if (std::abs(twist.twist.angular.z) < deadband) twist.twist.angular.z = 0.0;
    }

    void previousMovementFilterNoise(geometry_msgs::msg::TwistStamped& twist, const int& consecutive_nonzero) {
        geometry_msgs::msg::TwistStamped twist_copy(twist);
        for (const auto& prev : previous_proposed_twists_) {
            if (prev.twist.linear.x == 0.0) twist.twist.linear.x = 0.0;
            if (prev.twist.linear.y == 0.0) twist.twist.linear.y = 0.0;
            if (prev.twist.linear.z == 0.0) twist.twist.linear.z = 0.0;
            if (prev.twist.angular.x == 0.0) twist.twist.angular.x = 0.0;
            if (prev.twist.angular.y == 0.0) twist.twist.angular.y = 0.0;
            if (prev.twist.angular.z == 0.0) twist.twist.angular.z = 0.0;
        }
        previous_proposed_twists_.push_back(twist_copy);
    }

    void scaleTwist(geometry_msgs::msg::TwistStamped& twist, double scale) {
        twist.twist.linear.x *= scale;
        twist.twist.linear.y *= scale;
        twist.twist.linear.z *= scale;
        twist.twist.angular.x *= scale;
        twist.twist.angular.y *= scale;
        twist.twist.angular.z *= scale;
    }

    void transformTwistToUrFrame(geometry_msgs::msg::TwistStamped& twist) {
        double temp_x = twist.twist.linear.x;
        double temp_y = twist.twist.linear.y;

        twist.twist.linear.x = -temp_y;
        twist.twist.linear.y = temp_x;
    }

    rclcpp::Subscription<omni_msgs::msg::OmniState>::SharedPtr omni_sub_;
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr button_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    
    boost::circular_buffer<geometry_msgs::msg::TwistStamped> previous_proposed_twists_;

    rclcpp::Time last_processed_time_;
    geometry_msgs::msg::Quaternion last_orientation_;
    bool movement_active_ = false;

    double deadband_;
    double movement_scale_;
    int consecutive_nonzero_;
    static const int QUEUE_LENGTH = 10;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OmniStateToTwist>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
