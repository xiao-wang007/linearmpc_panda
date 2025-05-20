#pragma once

#include <controller_manager_msgs/ListControllers.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <linearmpc_panda/StampedFloat64MultiArray.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <mutex>
#include <std_msgs/Time.h>

namespace MPCControllers
{
    class MPCExecutorNode 
    {
    public:
        MPCExecutorNode();
        void run();

    private:
        //
        void fetch_init_u_ref(const std_msgs::Float64MultiArray::ConstPtr& msg);

        // Callback for receiving the MPC solution
        void mpc_sol_callback(const linearmpc_panda::StampedFloat64MultiArray::ConstPtr& msg);

        // Publish the upsampled u_cmd
        void publish_upsampled_command(const ros::TimerEvent& event);

        //
        void get_mpc_start_time(const std_msgs::Time::ConstPtr& msg);

        //
        void waitForControllerToBeRunning(const std::string& controller_name); 

        // ROS-related members
        ros::NodeHandle nh_;
        ros::Subscriber mpc_sol_sub_;
        ros::Subscriber mpc_start_time_sub_;
        ros::ServiceClient list_client_;

        // from latched publisher to cover the first bit before the first solution is ready
        ros::Subscriber init_u_ref_sub_; 

        ros::Publisher u_cmd_upsampled_pub_;
        ros::Timer upsample_timer_;
        std_msgs::Float64MultiArray upsampled_msg_;

        // Parameters
        double executor_frequency_ {1000.0}; // Frequency of the executor
        double h_mpc_ {0.04};
        int nu_ {7}; // Number of control inputs
        int Nh_ {4};
        int Nt_;

        //run time parameters 
        drake::trajectories::PiecewisePolynomial<double> u_cmd_spline_ {};
        Eigen::VectorXd u_cmd_now_;// Storage for the upsampled command
        int rows_ {}; // for reconstructing the matrix
        int cols_ {};
        Eigen::MatrixXd u_sol_; // Matrix to hold the MPC solution
        double current_time_ {};
        ros::Time t_mpc_start_ {};
        std::mutex sol_spline_mutex_;
        std::mutex mpc_t_mutex_;
    };

}