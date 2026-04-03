#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

#include <rclcpp_action/rclcpp_action.hpp>
#include "robot_interfaces/action/task_execute_cycle.hpp"

using namespace std::chrono_literals;

class HMIInterface : public rclcpp::Node {
public:
    using TaskExecuteCycle = robot_interfaces::action::TaskExecuteCycle;
    using GoalHandleTask = rclcpp_action::ClientGoalHandle<TaskExecuteCycle>;

    HMIInterface() : Node("hmi_interface") {

        // Publishers
        start_pub_ = this->create_publisher<std_msgs::msg::Bool>("/hmi/start_cycle", 10);
        stop_pub_ = this->create_publisher<std_msgs::msg::Bool>("/hmi/stop_cycle", 10);

        // Action client
        action_client_ = rclcpp_action::create_client<TaskExecuteCycle>(
            this, "/arm/task_execute_cycle");

        RCLCPP_INFO(this->get_logger(), "HMI interface initialized");
        
        //To be deleted. This send an action petition each 2 seconds. JUST FOR TESTING!!!
        timer_ = this->create_wall_timer(2s, std::bind(&HMIInterface::execute_cycle_action, this));

    }

    void execute_cycle_action() {

        if (!action_client_->wait_for_action_server(2s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        auto goal_msg = TaskExecuteCycle::Goal();

        RCLCPP_INFO(this->get_logger(), "Sending action goal...");

        rclcpp_action::Client<TaskExecuteCycle>::SendGoalOptions send_goal_options;

        send_goal_options.goal_response_callback =
            [this](GoalHandleTask::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleTask::SharedPtr,
                   const std::shared_ptr<const TaskExecuteCycle::Feedback> feedback) {
                (void)feedback;
                RCLCPP_INFO(this->get_logger(), "Feedback received");
            };

        send_goal_options.result_callback =
            [this](const GoalHandleTask::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "Cycle completed successfully");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Cycle failed");
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;

    rclcpp_action::Client<TaskExecuteCycle>::SharedPtr action_client_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HMIInterface>());
    rclcpp::shutdown();
    return 0;
}