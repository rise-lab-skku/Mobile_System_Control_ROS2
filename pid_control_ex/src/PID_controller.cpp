#include "pid_control_ex/PID_controller.h"

namespace mobile_system_control
{
    PIDController::PIDController() : Node("pid_controller")
    {
        // Get parameters
        this->declare_parameter<std::string>("path", "");
        this->get_parameter("path", data_path_);
        this->declare_parameter<double>("Kp", 0.5);
        this->get_parameter("Kp", Kp_);
        this->declare_parameter<double>("Ki", 0.1);
        this->get_parameter("Ki", Ki_);
        this->declare_parameter<double>("accel", 0.5);
        this->get_parameter("accel", throttle_);

        // Init subscriber
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/mobile_system_control/ego_vehicle", 10, std::bind(&PIDController::CarlaInputCallback, this, std::placeholders::_1));

        // Init publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/mobile_system_control/control_msg", 10);

        err_sum_ = 0;
        time_step_ = 0.025;

        // Read track data
        ReadPath();
    }

    void PIDController::ReadPath()
    {
        std::ifstream istr;
        double x, y;
        char space;
        PointXY temp;

        istr.open(data_path_, std::ios::in);

        if (!istr.is_open())
        {
            RCLCPP_FATAL(this->get_logger(), "Cannot open file: %s", data_path_.c_str());
        }

        while (istr >> x && istr >> space && istr >> y)
        {
            temp.x = x;
            temp.y = y;
            track_.push_back(temp);
        }

        istr.close();
    }

    void PIDController::CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        x_ = msg->data[0];
        y_ = msg->data[1];
        theta_ = msg->data[2];
        vel_ = msg->data[3];
        steer_ = msg->data[4];
        RunPID();
    }

    void PIDController::RunPID()
    {
        const int &idx_1 = (FindClosestIndex() + 1) % track_.size();
        const int &idx_2 = (idx_1 + 1) % track_.size();

        double ref_theta = atan2(track_[idx_2].y - track_[idx_1].y,
                                 track_[idx_2].x - track_[idx_1].x);

        double err = ref_theta - theta_;

        err = (err < -M_PI) ? err + 2 * M_PI : err;
        err = (err > M_PI) ? err - 2 * M_PI : err;

        // P control
        const double &p_control = Kp_ * err;

        // I control
        err_sum_ += err * time_step_; 
        const double i_control = Ki_ * err_sum_; 

        double control_cmd = p_control + i_control;
        control_cmd = std::min(1.0, std::max(-1.0, control_cmd));

        geometry_msgs::msg::Vector3Stamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "PID_example";
        cmd.vector.x = throttle_;
        cmd.vector.y = -control_cmd;
        cmd.vector.z = 0.0;
        publisher_->publish(cmd);
    }

    int PIDController::FindClosestIndex()
    {
        int closest_idx = 0;
        int cur_idx = 0;
        float dis = 10.0;
        for (auto &point : track_)
        {
            float temp_dis = sqrt(pow(x_ - point.x, 2) + pow(y_ - point.y, 2));
            if (temp_dis < dis)
            {
                dis = temp_dis;
                closest_idx = cur_idx;
            }
            cur_idx++;
        }

        return closest_idx;
    }
}
