#pragma once

#include "linear_mpc_prob.h"
#include <mutex>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <ros/ros.h>
#include <linearmpc_panda/StampedFloat64MultiArray.h>
#include <sensor_msgs/JointState.h>

#define NUM_JOINTS 7
#define PI 3.14

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::System;
using drake::AutoDiffXd;
using drake::math::InitializeAutoDiff;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::ModelInstanceIndex;
using drake::math::RollPitchYaw;
using drake::math::RigidTransform;
using drake::math::RotationMatrix;
using drake::multibody::Frame;
using drake::multibody::SpatialVelocity;
using drake::trajectories::PiecewisePolynomial;

using namespace drake; // for all eigen types
using namespace drake::solvers;

class MPCSolverNode {
public:
    //
    MPCSolverNode();

    /* will not work as this node is not exposed to controller interface 
       I may have to create a publisher for panda hardware states in my
       QPController interface */ 
    //MPCSolverNode(const Eigen::VectorXd& q_now,
                  //const Eigen::VectorXd& v_now);

    //
    ~MPCSolverNode() = default;
    
    //
    void run();

    // Function to solve the MPC problem
    void solve_and_update(const ros::TimerEvent& event);

    //
    void joint_state_callback_sim(const sensor_msgs::JointState::ConstPtr& msg);
    
    //
    void joint_state_callback_HW(const sensor_msgs::JointState::ConstPtr& msg);

private:
    // ROS related member variables
    ros::NodeHandle nh_;
    ros::Timer solver_timer_;
    ros::Publisher mpc_sol_pub_;
    ros::Subscriber state_sub_;
    linearmpc_panda::StampedFloat64MultiArray mpc_sol_msg_;

    //drake::systems::DiscreteStateIndex state_index_;
    bool do_sim_ {true}; 
    std::string panda_file_ {"/home/rosdrake/panda_arm.urdf"};
    std::string integrator_ {"RK4"};
    drake::math::RigidTransform<double> X_W_base_ {};
    int nx_ {14}; 
    int nq_ {7};
    int nu_ {7}; 
    int Nt_ {5}; 
    int n_exe_steps_ {1};
    double h_env_ {0.001}; //sim and panda control freq
    double h_mpc_{0.04};
    int Nh_ {};
    double execution_length_ {};
    double mpc_horizon_ {};
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_;
    Eigen::VectorXd udot_up_ {Eigen::VectorXd::Constant(nu_, 1000.0)}; 
    Eigen::VectorXd udot_low_ {-udot_up_}; 
    Eigen::MatrixXd xref_now_ {};
    Eigen::MatrixXd uref_now_ {};
    Eigen::Matrix<double, NUM_JOINTS, 1> q_now_ {};
    Eigen::Matrix<double, NUM_JOINTS, 1> v_now_ {};
    Eigen::Matrix<double, NUM_JOINTS, 1> u_now_ {};

    //member variables related to the reference traj
    int N_ = 60;
    std::vector<std::string> var_names_ = {"q_panda", "v_panda", "us", "fn", "ft", "v1_post", "w1_post", "h"};
    std::vector<int> dims_ = {7, 7, 7, 1, 2, 2, 1, 1};
    std::vector<int> times_ = {N_, N_, N_, 1, 1, 1, 1, N_-1};
    std::string ref_traj_path_ = "/home/rosdrake/src/src/mpc/traj_refs/1.npy";
    MyUtils::ProcessedSolution data_proc_; 

    AutoDiffVecXd f_grad_;
    PiecewisePolynomial<double> x_ref_spline_;
    PiecewisePolynomial<double> u_ref_spline_;

    /* Now I learnt, using unique_ptr avoids DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN assertion */
    std::unique_ptr<MyControllers::LinearMPCProb> prob_;

    // run-time member data
    ros::Time t_now_ {};
    //Eigen::VectorXd state_now_ {}; bad practic without initialization as size is not know, can't do << q_now_, v_now_ without resizing
    //Eigen::Matrix<double, NUM_JOINTS*2, 1> state_now_ {};  have to use macros since I cannot use nu_ here
    Eigen::VectorXd state_now_ {}; // initialize in constructor
    Eigen::MatrixXd u_ref_cmd_ {}; // initialize in constructor

    // Mutex for thread safety
    std::mutex mpc_mutex_;
    ros::Time t_init_node_ {};
};