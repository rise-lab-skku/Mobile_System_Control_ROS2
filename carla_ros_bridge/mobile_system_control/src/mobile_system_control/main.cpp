#include "rclcpp/rclcpp.hpp"
#include "mobile_system_control/mobile_system_control.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mobile_system_control::Carla>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}