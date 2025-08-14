#include "mobile_system_control/mobile_system_control.h"
#include <cmath>

namespace mobile_system_control
{
    // Helper functions remain the same
    const float &Min(const float &a, const float &b)
    {
        return a < b ? a : b;
    }

    const float &Max(const float &a, const float &b)
    {
        return a > b ? a : b;
    }

    Carla::Carla() : Node("mobile_system_control")
    {
        // initialize publishers
        pub_state2user_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("state2user", 10);
        pub_ctrl2carla_ = this->create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>("ctrl2carla", 10);

        // initialize subscribers
        sub_carla_ego_ = this->create_subscription<carla_msgs::msg::CarlaEgoVehicleStatus>(
            "carla_ego", 10, std::bind(&Carla::CarlaEgoCallback, this, _1));
        
        sub_carla_obj_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "carla_obj", 10, std::bind(&Carla::CarlaOdomCallback, this, _1));

        sub_user_ctrl_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "user_ctrl", 10, std::bind(&Carla::UserCtrlCallback, this, _1));

        setTopic();

        RCLCPP_INFO(this->get_logger(), "Mobile System Control Node has been initialized.");
    }

    void Carla::setTopic()
    {
        std_msgs::msg::MultiArrayDimension dim;
        dim.label = "x        [m]";
        dim.size = 1;
        state_.layout.dim.push_back(dim);
        dim.label = "y        [m]";
        dim.size = 1;
        state_.layout.dim.push_back(dim);
        dim.label = "theta    [rad]    (-pi ~ pi)";
        dim.size = 1;
        state_.layout.dim.push_back(dim);
        dim.label = "velocity [m/s]";
        dim.size = 1;
        state_.layout.dim.push_back(dim);
        dim.label = "steer    [rad]    (-1 ~ 1)";
        dim.size = 1;
        state_.layout.dim.push_back(dim);

        state_.layout.data_offset = 0;
        state_.data.resize(5, 0.0); // Resize data vector
    }

    void Carla::CarlaEgoCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg)
    {
        state_.data[3] = msg->velocity;
        state_.data[4] = msg->control.steer;
    }

    void Carla::CarlaOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        state_.data[0] = msg->pose.pose.position.x;
        state_.data[1] = msg->pose.pose.position.y;

        double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
        double cosy_cosp = 1 - 2 * (pow(msg->pose.pose.orientation.y, 2) + pow(msg->pose.pose.orientation.z, 2));
        state_.data[2] = atan2(siny_cosp, cosy_cosp);

        pub_state2user_->publish(state_);
    }

    void Carla::UserCtrlCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        auto ctrl_ = std::make_unique<carla_msgs::msg::CarlaEgoVehicleControl>();
        ctrl_->header = msg->header;
        
        ctrl_->throttle = Max(Min(static_cast<float>(msg->vector.x), 1.0f), 0.0f);
        ctrl_->steer = Max(Min(static_cast<float>(msg->vector.y), 1.0f), -1.0f);
        ctrl_->brake = Max(Min(static_cast<float>(msg->vector.z), 1.0f), 0.0f);
        ctrl_->gear = 1;
        ctrl_->hand_brake = false;
        ctrl_->reverse = false;
        ctrl_->manual_gear_shift = false;

        pub_ctrl2carla_->publish(std::move(ctrl_));
    }

} // namespace mobile_system_control