#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SerialBridge : public rclcpp::Node {
    public:
        SerialBridge();
    
    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        int serial_fd_;
};
