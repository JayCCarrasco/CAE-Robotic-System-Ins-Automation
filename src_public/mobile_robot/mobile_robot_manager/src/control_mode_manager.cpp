#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class ControlModeManager : public rclcpp::Node {
public:
  ControlModeManager() : Node("control_mode_manager"), current_mode_("STOP") {
    // Subscribers
    teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_teleop", 10, std::bind(&ControlModeManager::teleop_callback, this, _1));

    auto_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_auto", 10, std::bind(&ControlModeManager::auto_callback, this, _1));

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
	    "/control_mode", 10, std::bind(&ControlModeManager::control_callback, this, _1));

    //Publisher
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_selected", 10);

    //timer
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
    std::bind(&ControlModeManager::publish_cmd, this));
	
    RCLCPP_INFO(this->get_logger(), "Control Mode Manager started");
  }

private:
    void teleop_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        last_teleop_cmd_ = *msg;
    }

    void auto_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        last_auto_cmd_ = *msg;
    }

    void control_callback(const std_msgs::msg::String::SharedPtr msg){
        current_mode_ = msg->data;
    }

    void publish_cmd(){
        
        geometry_msgs::msg::Twist output;

        if(current_mode_ == "MANUAL"){
            output = last_teleop_cmd_;
        } else if (current_mode_ == "AUTO"){
            output = last_auto_cmd_;
        } else {
            output = geometry_msgs::msg::Twist();
        }

        cmd_pub_->publish(output);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist last_teleop_cmd_;
    geometry_msgs::msg::Twist last_auto_cmd_;
    std::string current_mode_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlModeManager>());
    rclcpp::shutdown();
    return 0;
}