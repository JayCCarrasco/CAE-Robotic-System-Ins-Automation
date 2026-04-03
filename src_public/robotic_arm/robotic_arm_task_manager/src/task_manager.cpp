#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <thread>
#include <chrono>

#include "robot_interfaces/action/task_execute_cycle.hpp"

using namespace std::chrono_literals;

class TaskManager : public rclcpp::Node {
public:
    using TaskExecuteCycle = robot_interfaces::action::TaskExecuteCycle;
	using GoalHandleTask = rclcpp_action::ServerGoalHandle<TaskExecuteCycle>;
	
	TaskManager() : Node("task_manager") {
		action_server_ = rclcpp_action::create_server<TaskExecuteCycle>(
		    this,
			"/arm/task_execute_cycle",
			std::bind(&TaskManager::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
			std::bind(&TaskManager::handle_cancel, this, std::placeholders::_1),
			std::bind(&TaskManager::handle_accepted, this, std::placeholders::_1)
			);
			
		RCLCPP_INFO(this->get_logger(), "Task Manager Action Server ready");
	}

private:
    rclcpp_action::Server<TaskExecuteCycle>::SharedPtr action_server_;
	
	//Validating objective
	rclcpp_action::GoalResponse handle_goal(
	    const rclcpp_action::GoalUUID &,
		std::shared_ptr<const TaskExecuteCycle::Goal> goal){
		
		RCLCPP_INFO(this->get_logger(), "Goal request received");
		//Code
		
		(void) goal;
		
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}
	
	rclcpp_action::CancelResponse handle_cancel(
	    const std::shared_ptr<GoalHandleTask> goal_handle){
		
	    RCLCPP_WARN(this->get_logger(), "Request received canceled");
        (void) goal_handle;

        return rclcpp_action::CancelResponse::ACCEPT;		
	}
	
	void handle_accepted(const std::shared_ptr<GoalHandleTask> goal_handle){
		std::thread{std::bind(&TaskManager::execute, this, std::placeholders::_1), goal_handle}.detach();	
	}
	
	void execute(const std::shared_ptr<GoalHandleTask> goal_handle){
		RCLCPP_INFO(this->get_logger(), "Executing task cycle");
		
		auto feedback = std::make_shared<TaskExecuteCycle::Feedback>();
		auto result = std::make_shared<TaskExecuteCycle::Result>();
		
		for (int i = 0; i <= 100; i += 10) {
			if (goal_handle->is_canceling()){
				result->success = false;
				goal_handle->canceled(result);
				RCLCPP_WARN(this->get_logger(), "Goal canceled during execution");
				return;
			}
			
			//Simulation of the robot
			std::this_thread::sleep_for(300ms);
		
			feedback->progress = i;
			goal_handle->publish_feedback(feedback);
			
			RCLCPP_INFO(this->get_logger(), "Progress: %d", i);		
		}
		
		result->success = true;
		
		goal_handle->succeed(result);
		
		RCLCPP_INFO(this->get_logger(), "Task cycle completed");	
	}
};

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TaskManager>());
	rclcpp::shutdown();
	return 0;
}


