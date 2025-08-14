#include "mpc_control_ex/MPC_controller.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mobile_system_control::MPCControler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}