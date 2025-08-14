#ifndef MPC_CONTROL_EX_MPC_CONTROLLER_H_
#define MPC_CONTROL_EX_MPC_CONTROLLER_H_

#include <cmath>
#include <memory>
#include <vector>
#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <OsqpEigen/OsqpEigen.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mobile_system_control
{
    class MPCControler : public rclcpp::Node
    {
    public:
        MPCControler();
        ~MPCControler();
        void CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr input);
        void ReadPath();
        void RunMPC();
        int FindClosestIndex();
        struct PointXY;

        void SetPath();
        void BuildMatrices();
        void SetReferences();
        void SetHessianMatrix();
        void SetGradient();
        void SetConstraintMatrix();
        void SetConstrainVectors();
        void KanayamaSteer();

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
}
#endif // MPC_CONTROL_EX_MPC_CONTROLLER_H_