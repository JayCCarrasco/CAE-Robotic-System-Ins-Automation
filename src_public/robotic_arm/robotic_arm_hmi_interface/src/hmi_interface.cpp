#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rclcpp_action/rclcpp_action.hpp>

#include "../robot_interfaces/action/TaskExecuteCycle.action"

using namespace std::chrono_literals;

class HMIInterface : public rclcpp::Node {
public:
    using TaskExecuteCycle = robot_interfaces::action::TaskExecuteCycle;
	using GoalHandleTask = rclcpp_action::ClientGoalHandle<TaskExecuteCycle>;
	
	HMIInterface() : Node("hmi_interface") {
	    //Publishers
		start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/hmi/start_cycle", 10);
		stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/hmi/stop_cycle", 10);
		
		//Action
		action_client_ = rclcpp_action::create_client<TaskExecuteCycle>(this, "/arm/task_execute_cycle");
		
		RCLCPP_INFO(this->get_logger(), "HMI interface initialized");
	}
	
	void start_cycle() {
	    auto msg = std_msgs::msg::Bool();
		msg.data = true;
		start_pub_->publish(msg);
		
		RCLCPP_INFO(this->get_logger(), "Start cycle command sent");
	}
	
	void stop_cycle() {
		auto msg = std_msgs::msg::Bool();
		msg.data = true;
		stop_pub_->publish(msg);
		
		RCLCPP_INFO(this->get_logger(), "Stop cycle command sent");
	}
	
	void execute_cycle_action() {
	    if(!action_client_->wait_for_action_server(2s)) {
		    RCLCPP_ERROR(this->get_logger(), "Action server not available");
			return;
		}
		
		auto goal_msg = TaskExecuteCycle::Goal();
		
		RCLCPP_INFO(this->get_logger(), "Sending action goal...");
		
		auto send_goal_options = rclcpp_action::Client<TaskExecuteCycle>::SendGoalOptions();
		
		send_goal_options.goal_response_callback = std::bind(&HMIInterface::goal_response_callback, this, std::placeholders::_1);
		
		send_goal_options.feedback_callback = std::bind(&HMIInterface::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
		
		send_goal_options.result_callback = std::bind(&HMIInterface::result_callback, this, std::placeholder::_1);
		
		action_client_->async_send_goal(goal_msg, send_goal_options);
		
		
		
	}
private:

    void goal_response_callback (std::shared_future<GoalHandleTask::SharedPtr> future) {
		auto goal_handle = future.get();
		
		if (!goal_handle) {
			RCLCPP_ERROR(this->get_logger(), "Goal rejected");
		} else {
			RCLCPP_INFO(this->get_logger(), "Goal accepted");
		}	
	}
	
	void feedback_callback(GoalHandleTask::SharedPtr, const std::shared_ptr<const TaskExecuteCycle::Feedback> feedback) {
		RCLCPP_INFO(this->get_logger(), "Feedback received");
		//Code to update status
	}
	
	void result_callback(const GoalHandleTask::WrappedResult& result) {
		if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
			RCLCPP_INFO(this->get_logger(), "Cycle completed successfully");
		} else {
			RCLCPP_ERROR(this->get_logger(), "Cycle failed");
		}
	}
	
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
	
	rclcpp_action::Client<TaskExecuteCycle>::SharedPtr action_client_;	
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<HMIInterface>());
	rclcpp::shutdown();
	return 0;
}