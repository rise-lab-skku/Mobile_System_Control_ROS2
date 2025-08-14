#ifndef MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_
#define MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_

#include "rclcpp/rclcpp.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_control.hpp"
#include "carla_msgs/msg/carla_ego_vehicle_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <memory>

// std::placeholders (e.g. _1)
using namespace std::placeholders;

namespace mobile_system_control
{
    class Carla : public rclcpp::Node
    {
    public:
        Carla();

    private:
        void CarlaEgoCallback(const carla_msgs::msg::CarlaEgoVehicleStatus::SharedPtr msg);
        void CarlaOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void UserCtrlCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
        void setTopic();
        
        // Publisher and Subscriber declarations
        rclcpp::Subscription<carla_msgs::msg::CarlaEgoVehicleStatus>::SharedPtr sub_carla_ego_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_carla_obj_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_user_ctrl_;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_state2user_;
        rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr pub_ctrl2carla_;
        
        std_msgs::msg::Float32MultiArray state_;
    };
} // namespace mobile_system_control

#endif // MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_MOBILESYSTEMCONTROL_H_