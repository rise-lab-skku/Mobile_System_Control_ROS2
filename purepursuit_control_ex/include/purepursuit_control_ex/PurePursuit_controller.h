#ifndef PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROLLER_H_
#define PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROLLER_H_

#include <cmath>
#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

namespace mobile_system_control
{
    class PurePursuitController : public rclcpp::Node
    {
    public:
        PurePursuitController();
        ~PurePursuitController();
        void CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr input);
        void ReadPath();
        void RunPurePursuit();
        int FindClosestIndex();
        struct PointXY;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROL_EX_PURE_PURSUIT_CONTROLLER_H_