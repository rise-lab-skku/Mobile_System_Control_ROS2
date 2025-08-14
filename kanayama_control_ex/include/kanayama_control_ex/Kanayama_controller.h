#ifndef KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_
#define KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_

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
    class KanayamaController : public rclcpp::Node
    {
    public:
        KanayamaController();
        ~KanayamaController();
        void CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr input);
        void ReadPath();
        void RunKanayama();
        int FindClosestIndex();
        struct PointXY;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // KANAYAMA_CONTROL_EX_KANAYAMA_CONTROL_EX_KANAYAMA_CONTROLLER_H_