/*Node balancing_robot_pid
by Juan Carlos del Pozo
12/01/2026
Description: Node that controls the balance for the balancing robot,
taking angle theta from the IMU and the reference velocity of the robot.
Then calculates the action of the motors through the PID and publish them.*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class BalancingRobotPID : public rclcpp::Node {
public:
    BalancingRobotPID() : Node("balancing_robot_pid"){
        kp_ = this->declare_parameter("kp", 71.16);
        ki_ = this->declare_parameter("ki", 0.0);
        kd_ = this->declare_parameter("kd", 5.21);

        control_frequency_ = this->declare_parameter("control_frequency", 200.0);
        dt_ = 1.0 / control_frequency_;

        // ----------------------------
        // Estado inicial
        // ----------------------------
        theta_ = 0.0;
        theta_dot_ = 0.0;
        theta_ref_ = 0.0;

        error_ = 0.0;
        last_error_ = 0.0;
        integral_ = 0.0;

        // ----------------------------
        // Subscriptores
        // ----------------------------
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&BalancingRobotPID::imu_callback, this, _1));
        // ----------------------------
        // Publicadores
        // ----------------------------
        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/left_wheel_effort", 10);

        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/right_wheel_effort", 10);
        // ----------------------------
        // Timer de control
        // ----------------------------
        timer_ = this->create_wall_timer(std::chrono::duration<double>(dt_), std::bind(&BalancingRobotPID::control_loop, this));
    
        RCLCPP_INFO(this->get_logger(), "Balancing robot PID node initialized");
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

    //void cmd_callback

    void control_loop(){
        //double Fmax = 20.0;

        error_ = theta_;

        integral_ += error_ * dt_;
        //double derivative_ = (error_ - last_error_) / dt_;
        last_error_ = error_;

        double effort = (kp_ * error_ +
                    ki_ * integral_+
                    kd_ * theta_dot_);
        
        publish_effort(effort);
    }

    void publish_effort(double effort){
        std_msgs::msg::Float64 msg;
        msg.data = effort;

        left_wheel_pub_->publish(msg);
        right_wheel_pub_->publish(msg);
    }

    //vars
    double kp_, ki_, kd_;
    double control_frequency_, dt_;

    double error_, last_error_, integral_;

    double theta_, theta_dot_, theta_ref_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BalancingRobotPID>());
    rclcpp::shutdown();
    return 0;
}

