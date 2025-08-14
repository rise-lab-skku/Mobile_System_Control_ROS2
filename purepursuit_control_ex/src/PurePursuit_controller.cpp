#include "purepursuit_control_ex/PurePursuit_controller.h"

namespace mobile_system_control
{
    struct PurePursuitController::PointXY
    {
        float x;
        float y;
    };

    float Distance(const PurePursuitController::PointXY &p0, const PurePursuitController::PointXY &p1)
    {
        return sqrt(pow((p0.x - p1.x), 2) + pow((p0.y - p1.y), 2));
    }

    struct PurePursuitController::Impl
    {
        // Subscriber
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_carla;

        // Publisher
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_cmd;

        // csv data path
        std::string data_path;

        // track vector
        std::vector<PointXY> track;

        // Input state
        float x;
        float y;
        float theta;
        float vel;
        float steer;

        // LD
        float LD;

        float L = 1.212;

        double err_sum;
        float time_step;
        float throttle = 0.8;
    };

    PurePursuitController::PurePursuitController() : Node("purepursuit_control_ex_node"), impl_(new Impl)
    {
        // Init subscriber
        impl_->sub_carla = this->create_subscription<std_msgs::msg::Float32MultiArray>("/mobile_system_control/ego_vehicle", 10, std::bind(&PurePursuitController::CarlaInputCallback, this, std::placeholders::_1));

        // Init publisher
        impl_->pub_cmd = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/mobile_system_control/control_msg", 10);

        // Get parameters
        this->declare_parameter<std::string>("path", "");
        this->get_parameter("path", impl_->data_path);
        this->declare_parameter<float>("LD", 0.0);
        this->get_parameter("LD", impl_->LD);

        impl_->err_sum = 0;
        impl_->time_step = 0.025;

        // Read track data
        ReadPath();
    }

    PurePursuitController::~PurePursuitController() {}

    void PurePursuitController::ReadPath()
    {
        std::ifstream istr;
        double x, y;
        char space;
        PointXY temp;

        istr.open(impl_->data_path, std::ios::in);

        if (!istr.is_open())
        {
            RCLCPP_FATAL(this->get_logger(), "Cannot open file: %s", impl_->data_path.c_str());
        }

        while (istr >> x && istr >> space && istr >> y)
        {
            temp.x = x;
            temp.y = y;
            impl_->track.push_back(temp);
        }

        istr.close();
    }

    void PurePursuitController::CarlaInputCallback(const std_msgs::msg::Float32MultiArray::SharedPtr input)
    {
        impl_->x = input->data[0];
        impl_->y = input->data[1];
        impl_->theta = input->data[2];
        impl_->vel = input->data[3];
        impl_->steer = input->data[4];
        RunPurePursuit();
    }

    void PurePursuitController::RunPurePursuit()
    {
        PointXY WL;
        PointXY intersection_point1, intersection_point2;

        const int &idx_1 = (FindClosestIndex() + 1) % impl_->track.size();
        const int &idx_2 = (idx_1 + 1) % impl_->track.size();

        PointXY n1 = impl_->track[idx_1];
        PointXY n2 = impl_->track[idx_2];

        // 이차방정식 근의공식으로 두개의 교점 찾기
        float slope = (n2.y - n1.y) / (n2.x - n1.x);
        if (std::isinf(slope) || slope > 1000)
        {
            intersection_point1.x = n2.x;
            intersection_point2.x = n2.x;
            intersection_point1.y = impl_->y + sqrt(-(pow(n1.x - impl_->x, 2) - pow(impl_->LD, 2)));
            intersection_point2.y = impl_->y - sqrt(-(pow(n1.x - impl_->x, 2) - pow(impl_->LD, 2)));
        }

        else
        {
            float a = 1 + pow(slope, 2);

            const float &b0 = -2 * impl_->x;
            const float &b1 = 2 * pow(slope, 2) * n1.x;
            const float &b2 = 2 * slope * (n1.y - impl_->y);
            float b = b0 - b1 + b2;

            const float &c0 = pow(impl_->x, 2);
            const float &c1 = pow(slope, 2) * pow(n1.x, 2);
            const float &c2 = 2 * slope * (n1.y - impl_->y) * n1.x;
            const float &c3 = pow(n1.y, 2) - 2 * n1.y * impl_->y;
            const float &c4 = pow(impl_->y, 2) - pow(impl_->LD, 2);
            float c = c0 + c1 - c2 + c3 + c4;

            intersection_point1.x = (-b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
            intersection_point2.x = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
            intersection_point1.y = slope * (intersection_point1.x - n1.x) + n1.y;
            intersection_point2.y = slope * (intersection_point2.x - n2.x) + n2.y;
        }

        // 두개의 교점중, 목표지점과 가까운 점을 WL에 저장
        const float &distance1 = Distance(n2, intersection_point1);
        const float &distance2 = Distance(n2, intersection_point2);

        WL = (distance1 > distance2) ? intersection_point2 : intersection_point1;

        // finding alpha
        const double &X = WL.x - impl_->x;
        const double &Y = WL.y - impl_->y;
        double path_heading = atan2(Y, X);
        double alpha = path_heading - impl_->theta;
        alpha = (alpha > M_PI) ? alpha - 2 * M_PI : alpha;
        alpha = (alpha < -M_PI) ? alpha + 2 * M_PI : alpha;

        // finding delta
        double e = impl_->LD * sin(fabs(alpha) * M_PI / 180);
        double delta = atan((2 * impl_->L * e) / (impl_->LD * impl_->LD)) * 180 / M_PI;
        if (alpha < 0)
        {
            delta *= -1;
        }

        delta /= 20.0 * M_PI / 180.0;

        delta = std::min(1.0, std::max(-1.0, delta));

        geometry_msgs::msg::Vector3Stamped cmd;
        cmd.header.frame_id = "PurePursuit_example";
        cmd.vector.x = impl_->throttle;
        cmd.vector.y = -delta;
        cmd.vector.z = 0.0;
        impl_->pub_cmd->publish(cmd);
    }

    int PurePursuitController::FindClosestIndex()
    {
        int closest_idx = 0;
        int cur_idx = 0;
        float dis = 10.0;
        for (auto &point : impl_->track)
        {
            float temp_dis = sqrt(pow(impl_->x - point.x, 2) + pow(impl_->y - point.y, 2));
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
