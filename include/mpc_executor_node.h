#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

class MPCExecutorNode {
public:
    MPCExecutorNode();
    void run();

private:
    // Callback for receiving the MPC solution
    void mpc_sol_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    // Publish the upsampled u_cmd
    void publish_upsampled_command(const ros::TimerEvent& event);

    // ROS-related members
    ros::NodeHandle nh_;
    ros::Subscriber mpc_sol_sub_;
    ros::Publisher u_cmd_upsampled_pub_;
    ros::Timer upsample_timer_;
    std_msgs::Float64MultiArray upsampled_msg_;

    // Drake's PiecewisePolynomial for interpolation
    drake::trajectories::PiecewisePolynomial<double> u_cmd_spline_;

    // Parameters
    double solver_frequency_; // Frequency of the MPC solver
    double executor_frequency_; // Frequency of the executor
    int nu_; // Number of control inputs

    // Storage for the upsampled command
    Eigen::VectorXd u_cmd_now_;
};