#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
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
    ros::Subscriber wrench_sub_;
    ros::Subscriber controller_start_sub_;
    ros::Subscriber traj_complete_sub_;
    ros::Timer timer1_;
    ros::Timer timer2_;
    std::string log_file_path_;
    double sample_rate_;
    bool first_message_ = true;
    ros::Time start_time_;
    bool controller_started_ = false;
    bool trajectory_completed_ = false;
    double controller_start_time_ = 0.0;

    // For thread safety
    std::mutex mutex_;
    sensor_msgs::JointState::ConstPtr latest_joint_msg_;
    geometry_msgs::WrenchStamped::ConstPtr latest_wrench_msg_;
    bool new_data_ = false;

    // In-memory storage
    std::vector<double> timestamps_joint_;
    std::vector<double> timestamps_wrench_;
    std::vector<std::vector<double>> positions_;
    std::vector<std::vector<double>> velocities_;
    std::vector<std::vector<double>> efforts_;
    std::vector<std::vector<double>> wrenches_;
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

        wrench_sub_ = nh_.subscribe("/franka_state_controller/F_ext", 
                                        1000,
                                        &JointStateLogger::wrenchCallback, 
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
        timer1_ = nh_.createTimer(ros::Duration(1.0/sample_rate_), 
                                &JointStateLogger::timerCallback_joint,
                                this);

        timer2_ = nh_.createTimer(ros::Duration(1.0/sample_rate_), 
                                &JointStateLogger::timerCallback_wrench,
                                this);
        
        ROS_INFO("Joint state logger initialized. Will log at %.1f Hz when controller starts.", 
                 sample_rate_);
        ROS_INFO("Waiting for controller to start...");
    }
    
    ~JointStateLogger() {
        if (!timestamps_joint_.empty() && !trajectory_completed_) {
            saveToFile_joint();
            saveToFile_wrench();
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
            saveToFile_joint();
            saveToFile_wrench();
            
            // Optionally shutdown the node
            ros::shutdown();
        }
    }
    
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        // Just store the latest message
        std::lock_guard<std::mutex> lock(mutex_);
        latest_joint_msg_ = msg;
        new_data_ = true;
    }

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        // Just store the latest message
        std::lock_guard<std::mutex> lock(mutex_);
        latest_wrench_msg_ = msg;
        new_data_ = true;
    }

    void timerCallback_joint(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Skip if controller hasn't started yet or trajectory is complete
        if (!controller_started_ || trajectory_completed_ || !new_data_ || !latest_joint_msg_)
            return;
            
        if (first_message_) {
            start_time_ = ros::Time(controller_start_time_);
            first_message_ = false;
        }
        
        // Process latest message
        double timestamp = (latest_joint_msg_->header.stamp - start_time_).toSec();
        
        // Only log data that occurs after controller start time
        if (timestamp >= 0.0) {
            timestamps_joint_.push_back(timestamp);
            positions_.push_back(latest_joint_msg_->position);
            velocities_.push_back(latest_joint_msg_->velocity);
            efforts_.push_back(latest_joint_msg_->effort);
            
            // For debugging
            if (timestamps_joint_.size() % 100 == 0) {
                ROS_INFO("Logged %zu samples for joint states", timestamps_joint_.size());
            }
        }
        
        new_data_ = false;
    }

    void timerCallback_wrench(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        // Skip if controller hasn't started yet or trajectory is complete
        if (!controller_started_ || trajectory_completed_ || !new_data_ || !latest_wrench_msg_)
            return;
            
        if (first_message_) {
            start_time_ = ros::Time(controller_start_time_);
            first_message_ = false;
        }
        
        // Process latest message
        double timestamp = (latest_wrench_msg_->header.stamp - start_time_).toSec();
        
        // Only log data that occurs after controller start time
        if (timestamp >= 0.0) {
            timestamps_wrench_.push_back(timestamp);

            // Extract force and torque into a single vector
            std::vector<double> wrench_data = {
                latest_wrench_msg_->wrench.force.x,
                latest_wrench_msg_->wrench.force.y,
                latest_wrench_msg_->wrench.force.z,
                latest_wrench_msg_->wrench.torque.x,
                latest_wrench_msg_->wrench.torque.y,
                latest_wrench_msg_->wrench.torque.z
            };
            wrenches_.push_back(wrench_data);
            
            // For debugging
            if (timestamps_wrench_.size() % 100 == 0) {
                ROS_INFO("Logged %zu samples for wrench data", timestamps_wrench_.size());
            }
        }
        
        new_data_ = false;
    }
    
    void saveToFile_joint() {
        if (timestamps_joint_.empty()) {
            ROS_WARN("No data to save");
            return;
        }
        
        ROS_INFO("Saving %zu data points to %s", timestamps_joint_.size(), log_file_path_.c_str());
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
        for (size_t i = 0; i < timestamps_joint_.size(); i++) {
            file << timestamps_joint_[i] << ",";
            
            for (double pos : positions_[i]) file << pos << ",";
            for (double vel : velocities_[i]) file << vel << ",";
            for (double eff : efforts_[i]) file << eff << ",";
            for (size_t j = 0; j < efforts_[i].size(); j++) {
                file << efforts_[i][j];
                if (j < efforts_[i].size() - 1) file << ",";
            }
            file << std::endl;
        }
        
        ROS_INFO("Joint states data saved successfully to %s", log_file_path_.c_str());
    }

    void saveToFile_wrench() {
        if (timestamps_wrench_.empty()) {
            ROS_WARN("No data to save");
            return;
        }
        
        ROS_INFO("Saving %zu data points to %s", timestamps_wrench_.size(), log_file_path_.c_str());
        std::ofstream file(log_file_path_);
        
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", log_file_path_.c_str());
            return;
        }
        
        // Write header
        file << "timestamp, fx_base, fy_base, fz_base, mx_base, my_base, mz_base" << std::endl;
        
        // Write data all at once
        for (size_t i = 0; i < timestamps_wrench_.size(); i++) {
            file << timestamps_wrench_[i] << ",";
            
            for (double entry : wrenches_[i]) file << entry << ",";
            
            for (size_t j = 0; j < efforts_[i].size(); j++) {
                file << efforts_[i][j];
                if (j < efforts_[i].size() - 1) file << ",";
            }
            file << std::endl;
        }
        
        ROS_INFO("Wrench data saved successfully to %s", log_file_path_.c_str());
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