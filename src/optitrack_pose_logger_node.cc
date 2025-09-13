#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <string>
#include <mutex>

class OptitrackPoseLogger {
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber controller_start_sub_;
    ros::Subscriber traj_complete_sub_;
    ros::Timer debug_timer_;
    
    std::string initial_pose_path_;
    std::string final_pose_path_;
    std::string pose_topic_;
    
    std::mutex pose_mutex_;
    geometry_msgs::PoseStamped::ConstPtr latest_pose_;
    
    bool start_pose_saved_ = false;
    bool final_pose_saved_ = false;
    geometry_msgs::PoseStamped start_pose_;
    ros::Time last_pose_update_time_;

public:
    OptitrackPoseLogger(ros::NodeHandle& nh) : nh_(nh) {
        // Get parameters
        nh_.param<std::string>("initial_pose_file", initial_pose_path_, "/home/rosdrake/initial_pose.csv");
        nh_.param<std::string>("final_pose_file", final_pose_path_, "/home/rosdrake/final_pose.csv");
        nh_.param<std::string>("pose_topic", pose_topic_, "/mocap/rigid_bodies/Tomato_soup/pose");
        
        // Subscribe to optitrack pose topic
        pose_sub_ = nh_.subscribe("/mocap/rigid_bodies/Tomato_soup/pose", 
                                 10,
                                 &OptitrackPoseLogger::poseCallback, 
                                 this);
        
        // Subscribe to controller start time
        controller_start_sub_ = nh_.subscribe("/controller_t_start", 
                                             1, 
                                             &OptitrackPoseLogger::controllerStartCallback,
                                             this);
        
        // Subscribe to trajectory completion
        traj_complete_sub_ = nh_.subscribe("/trajectory_completion",
                                          1,
                                          &OptitrackPoseLogger::trajectoryCompleteCallback,
                                          this);
        // Add a debug timer
        debug_timer_ = nh_.createTimer(ros::Duration(3.0), 
                                     &OptitrackPoseLogger::debugTimerCallback, 
                                     this);
        ROS_INFO("Optitrack pose logger initialized.");
        ROS_INFO("Will save initial pose to: %s", initial_pose_path_.c_str());
        ROS_INFO("Will save final pose to: %s", final_pose_path_.c_str());
    }

    void debugTimerCallback(const ros::TimerEvent&) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        ROS_INFO("Debug status:");
        ROS_INFO("  - Have pose data: %s", latest_pose_ ? "YES" : "NO");
        ROS_INFO("  - Start pose saved: %s", start_pose_saved_ ? "YES" : "NO");
        ROS_INFO("  - Final pose saved: %s", final_pose_saved_ ? "YES" : "NO");
        
        // Test topics existence
        ros::master::V_TopicInfo topic_info;
        ros::master::getTopics(topic_info);
        
        bool pose_topic_exists = false;
        bool start_topic_exists = false;
        bool complete_topic_exists = false;
        
        for (const auto& topic : topic_info) {
            if (topic.name == pose_topic_) pose_topic_exists = true;
            if (topic.name == "/controller_t_start") start_topic_exists = true;
            if (topic.name == "/trajectory_completion") complete_topic_exists = true;
        }
        
        ROS_INFO("Topic status:");
        ROS_INFO("  - Pose topic (%s): %s", pose_topic_.c_str(), pose_topic_exists ? "EXISTS" : "NOT FOUND");
        ROS_INFO("  - Controller start topic: %s", start_topic_exists ? "EXISTS" : "NOT FOUND");
        ROS_INFO("  - Trajectory completion topic: %s", complete_topic_exists ? "EXISTS" : "NOT FOUND");
        
        if (!start_topic_exists || !complete_topic_exists) {
            ROS_WARN("Controller topics not found. Is the controller running?");
        }
        
        if (!pose_topic_exists) {
            ROS_WARN("Mocap topic not found. Try these commands in separate terminals:");
            ROS_WARN("  rostopic list | grep mocap");
            ROS_WARN("  rostopic info /controller_t_start");
            ROS_WARN("  rostopic info /trajectory_completion");
        }
    }
    
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = msg;
        last_pose_update_time_ = ros::Time::now();
    }

    // Modify the controllerStartCallback
    void controllerStartCallback(const std_msgs::Float64::ConstPtr& msg) 
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if (!start_pose_saved_ && latest_pose_) {
            // Store a deep copy of the start pose
            start_pose_ = *latest_pose_;
            savePoseToFile(latest_pose_, initial_pose_path_);
            start_pose_saved_ = true;
            ROS_INFO("Initial pose saved to: %s at time %f", 
                    initial_pose_path_.c_str(), 
                    latest_pose_->header.stamp.toSec());
        }
    }


    void trajectoryCompleteCallback(const std_msgs::Bool::ConstPtr& msg) 
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        
        if (!final_pose_saved_ && latest_pose_) {
            // Check if we've received a new pose since saving the start pose
            bool is_new_pose = false;
            if (start_pose_saved_) {
                double time_diff = latest_pose_->header.stamp.toSec() - start_pose_.header.stamp.toSec();
                double pos_diff = 
                    std::abs(latest_pose_->pose.position.x - start_pose_.pose.position.x) +
                    std::abs(latest_pose_->pose.position.y - start_pose_.pose.position.y) +
                    std::abs(latest_pose_->pose.position.z - start_pose_.pose.position.z);
                    
                is_new_pose = (time_diff > 0.01) || (pos_diff > 0.001);
            }
            
            if (!is_new_pose) {
                ROS_WARN("Warning: Final pose appears to be the same as initial pose!");
                ROS_WARN("This may indicate that no new OptiTrack data was received during execution.");
            }
            
            savePoseToFile(latest_pose_, final_pose_path_);
            final_pose_saved_ = true;
            ROS_INFO("Final pose saved to: %s at time %f", 
                    final_pose_path_.c_str(), 
                    latest_pose_->header.stamp.toSec());
            
            // Since both poses are saved, we can shut down
            if (start_pose_saved_) {
                ros::shutdown();
            }
        }
    }

    
    void savePoseToFile(const geometry_msgs::PoseStamped::ConstPtr& pose, 
                        const std::string& filepath) {
        std::ofstream file(filepath);
        
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", filepath.c_str());
            return;
        }
        
        // Write header
        file << "timestamp,frame_id,pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w" << std::endl;
        
        // Write data
        file << pose->header.stamp << "," 
             << pose->header.frame_id << ","
             << pose->pose.position.x << "," 
             << pose->pose.position.y << ","
             << pose->pose.position.z << ","
             << pose->pose.orientation.x << ","
             << pose->pose.orientation.y << ","
             << pose->pose.orientation.z << ","
             << pose->pose.orientation.w;
        
        file.close();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "optitrack_pose_logger");
    ros::NodeHandle nh("~");
    
    OptitrackPoseLogger logger(nh);
    
    ROS_INFO("Optitrack pose logger running - waiting for controller signals");
    ros::spin();
    
    return 0;
}