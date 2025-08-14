#ifndef PID_CONTROL_EX_PID_CONTROL_EX_PID_CONTROLLER_H_
#define PID_CONTROL_EX_PID_CONTROL_EX_PID_CONTROLLER_H_

#include <cmath>
#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

namespace mobile_system_control
{
    class PIDController : public rclcpp::Node
    {
    public:
        PIDController();

    private:
        void CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void ReadPath();
        void RunPID();
        int FindClosestIndex();

        struct PointXY
        {
            float x;
            float y;
        };

        // Subscriber
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;

        // Publisher
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr publisher_;

        // csv data path
        std::string data_path_;

        // track vector
        std::vector<PointXY> track_;

        // Input state
        float x_;
        float y_;
        float theta_;
        float vel_;
        float steer_;

        // PI Gain
        float Kp_;
        float Ki_;

        double err_sum_;
        float time_step_;
        float throttle_;
    };
}
#endif // PID_CONTROL_EX_PID_CONTROL_EX_PID_CONTROLLER_H_
