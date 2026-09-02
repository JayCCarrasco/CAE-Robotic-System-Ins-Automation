#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <fstream>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;


class DataLogger : public rclcpp::Node {
public:
    DataLogger() : Node("data_logger"){

        //Subscribers
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&DataLogger::imu_callback, this, _1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/world/balancing_robot_world/model/balancing_robot/joint_state", 10, std::bind(&DataLogger::joint_callback, this, _1));

        effort_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/balancing_robot/effort", 10, std::bind(&DataLogger::effort_callback, this, _1));
        
        //timer
        timer_ = this->create_wall_timer(5ms,
            std::bind(&DataLogger::log_data, this));

        //file management
        file_.open("balancing_robot_dataset.csv");
        if (!file_.is_open()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Could not open dataset file");
        }
        file_ << "timestamp, theta, theta_dot, x, x_dot, effort\n";
    }

    ~DataLogger(){
        if (file_.is_open()) {
            file_.close();
        }
    }

private:

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){

        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w; //Scalar part of the quaternion 

        theta_ = std::atan2(
            2.0 * (qw * qy + qx * qz),
            1.0 - 2.0 * (qy*qy + qx * qx));

        theta_dot_ = msg->angular_velocity.y;
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        double omega_left = 0.0;
        double omega_right = 0.0;

        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "left_wheel_joint") {
                omega_left = msg->velocity[i];
            }
            if (msg->name[i] == "right_wheel_joint") {
                omega_right = msg->velocity[i];
            }
        }

        double r = 0.05;
        x_dot_ = r * (omega_left + omega_right) / 2.0;
        //x_ += x_dot_ * dt_;
    }

    void effort_callback(const std_msgs::msg::Float64::SharedPtr msg){
        effort_ = msg->data;
    }

    void log_data(){
        x_ += x_dot_ * dt_;

        auto now = this->get_clock()->now();

        file_
            << now.seconds() << ","
            << theta_ << ","
            << theta_dot_ << ","
            << x_ << ","
            << x_dot_ << ","
            << effort_
            << "\n";
    }

    //vars
    double  theta_ = 0;
    double theta_dot_ = 0;
    double x_ = 0;
    double x_dot_ = 0;
    double dt_ = 1.0/200.0;
    double effort_ = 0;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr effort_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream file_;
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataLogger>());
    rclcpp::shutdown();
    return 0;
}