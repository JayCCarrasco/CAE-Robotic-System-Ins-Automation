#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <string>

using std::placeholders::_1;

class SerialBridge : public rclcpp::Node {
    public:
        SerialBridge() : Node("serial_bridge"){
            cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_selected",
            10,
            std::bind(&SerialBridge::cmd_vel_callback, this, _1));

            serial_fd_ = -1;
            port_ = "dev/ttyUSB0";
            baudrate_ = B115200;

            open_serial_port();

            RCLCPP_INFO(this->get_logger(),"Serial bridge node started");
        }

        ~SerialBridge(){
            close_serial_port();
        }
    
    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
            RCLCPP_INFO(
                this->get_logger(),
                "Linear: %.3f Angular: %.3f",
                msg->linear.x,
                msg->angular.z);
        }

        bool open_serial_port(){
            serial_fd_ = open(port_.c_str(), O_RDWR| O_NOCTTY | O_SYNC);

            if (serial_fd_ < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port_.c_str());

                return false;
            }

            RCLCPP_INFO(
                this->get_logger(), "Serial port %s opened succesfully", port_.c_str());
            
            return true;
        }

        void close_serial_port(){
            if(serial_fd_ >= 0){
                close(serial_fd_);
                serial_fd_ = -1;
            }
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
        int serial_fd_;
        std::string port_;
        speed_t baudrate_;

};

int main(int argc, char *argv []){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridge>());
    rclcpp::shutdown();

    return 0;
}
