#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include <cmath>
#include <fstream>
#include <chrono>
#include <deque>

using namespace std::chrono_literals;
using std::placeholders::_1;


class DataLogger : public rclcpp::Node {
public:

    DataLogger() : Node("data_logger") {

        // =========================
        // Subscribers
        // =========================

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu",
            10,
            std::bind(&DataLogger::imu_callback, this, _1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/world/balancing_robot_world/model/balancing_robot/joint_state",
            10,
            std::bind(&DataLogger::joint_callback, this, _1));

        effort_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/balancing_robot/effort",
            10,
            std::bind(&DataLogger::effort_callback, this, _1));


        // =========================
        // Timer: 200 Hz
        // =========================

        timer_ = this->create_wall_timer(
            5ms,
            std::bind(&DataLogger::log_data, this));


        // =========================
        // File
        // =========================

        file_.open("balancing_robot_dataset.csv");

        if (!file_.is_open()) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Could not open dataset file");
        }

        file_
            << "timestamp,theta,theta_dot,x,x_dot,effort,episode\n";


        RCLCPP_INFO(
            this->get_logger(),
            "DataLogger initialized at 200 Hz");

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for perturbation...");
    }


    ~DataLogger() {

        if (file_.is_open()) {
            file_.close();
        }
    }


private:

    // ============================================================
    // Sample structure
    // ============================================================

    struct Sample {

        int64_t timestamp;

        double theta;
        double theta_dot;

        double x;
        double x_dot;

        double effort;
    };


    // ============================================================
    // IMU
    // ============================================================

    void imu_callback(
        const sensor_msgs::msg::Imu::SharedPtr msg) {

        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        theta_ = std::atan2(
            2.0 * (qw * qy + qx * qz),
            1.0 - 2.0 * (qy * qy + qx * qx));

        theta_dot_ = msg->angular_velocity.y;
    }


    // ============================================================
    // Joint states
    // ============================================================

    void joint_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg) {

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
    }


    // ============================================================
    // Effort
    // ============================================================

    void effort_callback(
        const std_msgs::msg::Float64::SharedPtr msg) {

        effort_ = msg->data;
    }


    // ============================================================
    // Logger
    // ============================================================

    void log_data() {

        auto now = this->get_clock()->now();
        int64_t timestamp_ns = now.nanoseconds();


        // --------------------------------------------------------
        // Calculate dt
        // --------------------------------------------------------

        double dt = 0.0;

        if (last_timestamp_ns_ != 0) {

            dt = (timestamp_ns - last_timestamp_ns_) * 1e-9;

            x_ += x_dot_ * dt;
        }

        last_timestamp_ns_ = timestamp_ns;


        // --------------------------------------------------------
        // Current sample
        // --------------------------------------------------------

        Sample sample;

        sample.timestamp = timestamp_ns;
        sample.theta = theta_;
        sample.theta_dot = theta_dot_;
        sample.x = x_;
        sample.x_dot = x_dot_;
        sample.effort = effort_;


        // --------------------------------------------------------
        // Always maintain the pre-event buffer
        // --------------------------------------------------------

        if (!recording_) {

            pre_buffer_.push_back(sample);

            if (pre_buffer_.size() > PRE_BUFFER_SIZE) {
                pre_buffer_.pop_front();
            }
        }


        // ========================================================
        // STATE: IDLE
        // ========================================================

        if (!recording_) {

            bool perturbation_detected =
                std::abs(theta_) > THETA_TRIGGER ||
                std::abs(theta_dot_) > THETA_DOT_TRIGGER;


            if (perturbation_detected) {

                // -----------------------------------------------
                // New episode
                // -----------------------------------------------

                recording_ = true;

                episode_id_++;

                stable_start_ns_ = 0;


                RCLCPP_INFO(
                    this->get_logger(),
                    "Perturbation detected -> episode %d",
                    episode_id_);


                // -----------------------------------------------
                // Write previous samples
                // -----------------------------------------------

                for (const auto &previous_sample : pre_buffer_) {

                    write_sample(
                        previous_sample,
                        episode_id_);
                }

                pre_buffer_.clear();


                // -----------------------------------------------
                // Write current sample
                // -----------------------------------------------

                write_sample(sample, episode_id_);
            }

            return;
        }


        // ========================================================
        // STATE: RECORDING
        // ========================================================

        write_sample(sample, episode_id_);


        // --------------------------------------------------------
        // Check whether robot is stable
        // --------------------------------------------------------

        bool stable =
            std::abs(theta_) < THETA_STABLE &&
            std::abs(theta_dot_) < THETA_DOT_STABLE;


        if (stable) {

            // First stable sample
            if (stable_start_ns_ == 0) {
                stable_start_ns_ = timestamp_ns;
            }


            double stable_time =
                (timestamp_ns - stable_start_ns_) * 1e-9;


            // ----------------------------------------------------
            // Stable for required time -> episode finished
            // ----------------------------------------------------

            if (stable_time >= STABLE_TIME) {

                RCLCPP_INFO(
                    this->get_logger(),
                    "Episode %d finished. Robot stable.",
                    episode_id_);


                recording_ = false;

                stable_start_ns_ = 0;

                pre_buffer_.clear();


                RCLCPP_INFO(
                    this->get_logger(),
                    "Waiting for next perturbation...");
            }

        } else {

            // Robot became unstable again.
            // Reset stability timer.

            stable_start_ns_ = 0;
        }
    }


    // ============================================================
    // Write sample
    // ============================================================

    void write_sample(
        const Sample &sample,
        int episode) {

        file_
            << sample.timestamp << ","
            << sample.theta << ","
            << sample.theta_dot << ","
            << sample.x << ","
            << sample.x_dot << ","
            << sample.effort << ","
            << episode
            << "\n";
    }

    // ============================================================
    // Parameters
    // ============================================================

    // Perturbation detection
    const double THETA_TRIGGER = 0.03;          // rad ≈ 1.7°
    const double THETA_DOT_TRIGGER = 0.05;      // rad/s


    // Stability detection
    const double THETA_STABLE = 0.01;           // rad ≈ 0.57°
    const double THETA_DOT_STABLE = 0.05;       // rad/s

    const double STABLE_TIME = 0.5;             // seconds


    // 200 Hz -> 100 samples = 0.5 seconds
    const size_t PRE_BUFFER_SIZE = 100;


    // ============================================================
    // Variables
    // ============================================================

    double theta_ = 0.0;
    double theta_dot_ = 0.0;

    double x_ = 0.0;
    double x_dot_ = 0.0;

    double effort_ = 0.0;

    int64_t last_timestamp_ns_ = 0;

    int64_t stable_start_ns_ = 0;


    // Episode management
    bool recording_ = false;

    int episode_id_ = 0;


    // Previous samples
    std::deque<Sample> pre_buffer_;


    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr effort_sub_;

    rclcpp::TimerBase::SharedPtr timer_;


    // File
    std::ofstream file_;
};


int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<DataLogger>());

    rclcpp::shutdown();

    return 0;
}