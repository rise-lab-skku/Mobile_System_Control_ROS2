#include "pid_control_ex/PID_controller.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mobile_system_control::PIDController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
