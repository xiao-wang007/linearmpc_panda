#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <fstream>
#include <signal.h>
#include <mutex>

class JointStateLogger {
private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber controller_start_sub_;
    ros::Subscriber traj_complete_sub_;
    ros::Timer timer_;
    std::string log_file_path_;
    double sample_rate_;
    bool first_message_ = true;
    ros::Time start_time_;
    bool controller_started_ = false;
    bool trajectory_completed_ = false;
    double controller_start_time_ = 0.0;

    // For thread safety
    std::mutex mutex_;
    sensor_msgs::JointState::ConstPtr latest_msg_;
    bool new_data_ = false;

    // In-memory storage
    std::vector<double> timestamps_;
    std::vector<std::vector<double>> positions_;
    std::vector<std::vector<double>> velocities_;
    std::vector<std::vector<double>> efforts_;

public:
    JointStateLogger(ros::NodeHandle& nh) : nh_(nh) {
        // Get parameters
        nh_.param<std::string>("log_file", log_file_path_, "/home/rosdrake/joint_states.csv");
        nh_.param<double>("sample_rate", sample_rate_, 500.0); // 500Hz by default

        // Subscribe to joint states topic
        joint_state_sub_ = nh_.subscribe("/franka_state_controller/joint_states", 
                                        1000,
                                        &JointStateLogger::jointStateCallback, 
                                        this);

        // Subscribe to controller start time
        controller_start_sub_ = nh_.subscribe("/controller_t_start", 
                                             1, 
                                             &JointStateLogger::controllerStartCallback,
                                             this);
        
        // Subscribe to trajectory completion
        traj_complete_sub_ = nh_.subscribe("/trajectory_completion",
                                          1,
                                          &JointStateLogger::trajectoryCompleteCallback,
                                          this);
        
        // Create timer for downsampling at desired rate
        timer_ = nh_.createTimer(ros::Duration(1.0/sample_rate_), 
                                &JointStateLogger::timerCallback,
                                this);
        
        ROS_INFO("Joint state logger initialized. Will log at %.1f Hz when controller starts.", 
                 sample_rate_);
        ROS_INFO("Waiting for controller to start...");
    }
    
    ~JointStateLogger() {
        if (!timestamps_.empty() && !trajectory_completed_) {
            saveToFile();
        }
    }

void controllerStartCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!controller_started_) {
        // Check if message is too old (more than 10 seconds)
        if (ros::Time::now().toSec() - msg->data > 10.0) {
            ROS_WARN("Ignoring stale controller start signal (%.2f seconds old)", 
                     ros::Time::now().toSec() - msg->data);
            return;
        }
        
        controller_started_ = true;
        controller_start_time_ = msg->data;
        ROS_INFO("Received controller start signal. Timestamp: %.3f", controller_start_time_);
        ROS_INFO("Starting to log joint states...");
    }
}

    void trajectoryCompleteCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && !trajectory_completed_) {
            std::lock_guard<std::mutex> lock(mutex_);
            trajectory_completed_ = true;
            ROS_INFO("Received trajectory completion signal. Stopping logging.");
            
            // Save data to file
            saveToFile();
            
            // Optionally shutdown the node
            ros::shutdown();
        }
    }
    
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // Just store the latest message
        std::lock_guard<std::mutex> lock(mutex_);
        latest_msg_ = msg;
        new_data_ = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Skip if controller hasn't started yet or trajectory is complete
        if (!controller_started_ || trajectory_completed_ || !new_data_ || !latest_msg_)
            return;
            
        if (first_message_) {
            start_time_ = ros::Time(controller_start_time_);
            first_message_ = false;
        }
        
        // Process latest message
        double timestamp = (latest_msg_->header.stamp - start_time_).toSec();
        
        // Only log data that occurs after controller start time
        if (timestamp >= 0.0) {
            timestamps_.push_back(timestamp);
            positions_.push_back(latest_msg_->position);
            velocities_.push_back(latest_msg_->velocity);
            efforts_.push_back(latest_msg_->effort);
            
            // For debugging
            if (timestamps_.size() % 100 == 0) {
                ROS_INFO("Logged %zu samples", timestamps_.size());
            }
        }
        
        new_data_ = false;
    }
    
    void saveToFile() {
        if (timestamps_.empty()) {
            ROS_WARN("No data to save");
            return;
        }
        
        ROS_INFO("Saving %zu data points to %s", timestamps_.size(), log_file_path_.c_str());
        std::ofstream file(log_file_path_);
        
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", log_file_path_.c_str());
            return;
        }
        
        // Write header
        file << "timestamp,";
        for (int i = 0; i < 7; i++) file << "position_" << i << ",";
        for (int i = 0; i < 7; i++) file << "velocity_" << i << ",";
        for (int i = 0; i < 7; i++) {
            file << "effort_" << i;
            if (i < 6) file << ",";
        }
        file << std::endl;
        
        // Write data all at once
        for (size_t i = 0; i < timestamps_.size(); i++) {
            file << timestamps_[i] << ",";
            
            for (double pos : positions_[i]) file << pos << ",";
            for (double vel : velocities_[i]) file << vel << ",";
            
            for (size_t j = 0; j < efforts_[i].size(); j++) {
                file << efforts_[i][j];
                if (j < efforts_[i].size() - 1) file << ",";
            }
            file << std::endl;
        }
        
        ROS_INFO("Data saved successfully to %s", log_file_path_.c_str());
    }
};

// Global pointer for signal handler
JointStateLogger* g_logger = nullptr;

void signalHandler(int sig) {
    ROS_INFO("Shutdown signal received, saving data...");
    if (g_logger) delete g_logger;
    ros::shutdown();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_logger", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    
    // Set up signal handler for clean shutdown
    signal(SIGINT, signalHandler);
    
    g_logger = new JointStateLogger(nh);
    
    ROS_INFO("Joint state logger running - Will start/stop with controller signals");
    ros::spin();
    
    return 0;


}