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
            port_ = "/dev/ttyUSB0";
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
            
            std::string cmd = "V," + 
                std::to_string(msg->linear.x) +
                "," + std::to_string(msg->angular.z) + "\n";
            
            ssize_t bytes_written = write(serial_fd_, cmd.c_str(), cmd.size());
            
            if (bytes_written < 0){
                RCLCPP_ERROR(this->get_logger(), "Error writing to serial port %s", strerror(errno));
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent: %s", cmd.c_str());
            }

            char buffer[256];

            ssize_t bytes_read = read(serial_fd_, buffer, sizeof(buffer) - 1);



            if(bytes_read > 0){
                buffer[bytes_read] = '\0';

                RCLCPP_INFO(this->get_logger(), "Arduino %s", buffer);
            }
        }

        bool open_serial_port(){
            serial_fd_ = open(port_.c_str(), O_RDWR| O_NOCTTY | O_SYNC);

            if (serial_fd_ < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port_.c_str());

                return false;
            }
            
            //Declaring termios struct
            struct termios tty;

            //checking if copying struct from serial_fd was fine
            if (tcgetattr(serial_fd_, &tty) != 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to get serial attributes %s", strerror(errno));
                
                close(serial_fd_);
                serial_fd_ = -1;

                return false;
            }

            //setting baudrates
            cfsetospeed(&tty, baudrate_);
            cfsetispeed(&tty, baudrate_);

            //setting device control from serial port
            // REMEMBER:
            //&= ~ for deactivate bit
            //|= for activate bit
            tty.c_cflag &= ~PARENB;     
            tty.c_cflag &= ~CSTOPB;
            tty.c_cflag &= ~CSIZE;
            tty.c_cflag |= CS8;

            //Activating receptor and local mode
            tty.c_cflag |= (CLOCAL | CREAD);

            //Deactivating hardware flow control
            tty.c_cflag &= ~CRTSCTS;
            
            //Deactivating software flow control
            tty.c_iflag &= ~(IXON | IXOFF | IXANY);
            tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

            //Deactivating exit process
            tty.c_oflag &= ~OPOST;

            //Configure control characters
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 10;
            
            if(tcsetattr(serial_fd_, TCSANOW, &tty) != 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to set serial attributes: %s", strerror(errno));
                
                close(serial_fd_);
                serial_fd_ = -1;

                return false;
            }

            RCLCPP_INFO(
                this->get_logger(), "Serial port %s configured succesfully", port_.c_str());
            
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
