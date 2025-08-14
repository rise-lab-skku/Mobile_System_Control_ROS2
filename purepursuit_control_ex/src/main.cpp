#include "purepursuit_control_ex/PurePursuit_controller.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char **argv)
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    // PurePursuitController 노드를 생성
    auto node = std::make_shared<mobile_system_control::PurePursuitController>();
    
    // 노드를 실행하고 콜백 함수 등을 처리
    rclcpp::spin(node);
    
    // 종료
    rclcpp::shutdown();
    return 0;
}