#pragma once

#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <linearmpc_panda/StampedFloat64MultiArray.h>
#include <ros/ros.h>

namespace MPCControllers 
{

	using drake::trajectories::PiecewisePolynomial;

    class MPCRefUpdaterNode
    {
    public:
        MPCRefUpdaterNode();
        void run();
    private:
        // publish time indexed ref traj
        void publish_ref_traj(const ros::TimerEvent& event);

        // ROS-related members
        ros::NodeHandle nh_;
        ros::Publisher ref_pub_;
        // ros::Timer upsample_timer_;
        linearmpc_panda::StampedFloat64MultiArray ref_traj_msg_;
        ros::Time t_stamp_ {}; 

        // Parameters
        double h_mpc_ {0.04};
        int nu_ {7}; // Number of control inputs
        int Nt_ {5};
        int Nh_ {};

        drake::trajectories::PiecewisePolynomial<double> x_ref_spline_;
        drake::trajectories::PiecewisePolynomial<double> u_ref_spline_;
        Eigen::VectorXd x_ref_now_; // Storage sampled reference trajectory
        Eigen::VectorXd u_ref_now_; 
    };


} // namespace MPCControllers