#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

constexpr double PI = 3.141592653589793;

class SafetyMonitor : public rclcpp::Node{
 
public:
    SafetyMonitor() : Node("safety_monitor"), obstacle_close_(false){
		//parameters
		min_distance_ = this->declare_parameter("obstacle_min_dist", 0.5);
		cmd_timeout_ = this->declare_parameter("cmd_timeout", 0.5);
		max_front_angle_deg_ = this->declare_parameter("max_front_angle_deg", 30.0);
		
		//subscribers
		vel_sel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
		    "/cmd_vel_selected", 10, std::bind(&SafetyMonitor::vel_sel_callback, this, _1));
			
		lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		    "/scan", 10, std::bind(&SafetyMonitor::lidar_callback, this, _1));

		imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
		    "/imu", 10, std::bind(&SafetyMonitor::imu_callback, this, _1));
			
		//Publisher
		vel_safe_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
		    "/cmd_vel_safe", 10);
			
		timer_ = this->create_wall_timer(
		    std::chrono::milliseconds(50),
			std::bind(&SafetyMonitor::publish_cmd, this));
			
		last_cmd_time_ = this->now();
			
		RCLCPP_INFO(this->get_logger(), "Safety Monitor initialized");
	}
	
private:
    void vel_sel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
		vel_safe_ = *msg;
		last_cmd_time_ = this->now();
	}
	
	void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
		obstacle_close_ = false;

        int center = msg->ranges.size() / 2;

        int half_width = static_cast<int>((max_front_angle_deg_ * PI / 180.0) / msg->angle_increment / 2.0);

        for (int i = center - half_width; i < center + half_width; i++){
            if (i > 0 && i < static_cast<int>(msg->ranges.size())){
                if (msg->ranges[i] < min_distance_){
                    obstacle_close_ = true;
                    return;
                }
            }
        }
	}
	
	void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
		imu_data_ = *msg;
	}
	
	void publish_cmd(){
        geometry_msgs::msg::Twist output;
		bool timeout = (this->now() - last_cmd_time_).seconds() > cmd_timeout_;

        if(obstacle_close_ || timeout){
            output = geometry_msgs::msg::Twist();
        } else {
            output = vel_safe_;
        }

        vel_safe_pub_->publish(output);

	}
	
	//Members
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sel_sub_;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_safe_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	
	geometry_msgs::msg::Twist vel_safe_;
    sensor_msgs::msg::Imu imu_data_;
	rclcpp::Time last_cmd_time_;
	
	bool obstacle_close_;
	double min_distance_;
	double cmd_timeout_;
	double max_front_angle_deg_;
	
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}