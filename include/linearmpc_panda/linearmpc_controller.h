#pragma once

#include <controller_manager_msgs/ListControllers.h>
#include "linear_mpc_prob.h"
#include <unsupported/Eigen/KroneckerProduct>
#include <sensor_msgs/JointState.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <linearmpc_panda/StampedFloat64MultiArray.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <mutex>
#include <std_msgs/Time.h>
#include "myutils.h"

namespace MyControllers
{
    // Constants
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

    class LinearMPCControllerNode
    {
    public:
        // Constructor
        LinearMPCControllerNode();

        // Destructor
        ~LinearMPCControllerNode() = default;

        void run();

        void solve_and_update();
        
        void publish_upsampled_command(const ros::TimerEvent& event);

        void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
    
    private:
        // ROS related member variables
        ros::NodeHandle nh_;
        ros::Publisher upsampled_u_cmd_pub_;

        ros::Subscriber state_sub_;
        ros::Time mpc_t_start_;
        Eigen::MatrixXd latest_mpc_sol_;

        std::string panda_file_ {"/home/rosdrake/src/drake_models/franka_description/urdf/panda_arm.urdf"};
        bool exclude_gravity_from_traj_ {true}; // as panda has internal gravity compensation
        std::string integrator_ {"Euler"};
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
        //std::vector<std::string> var_names_ = {"q_panda", "v_panda", "us", "fn", "ft", "v1_post", "w1_post", "h"};
        //std::vector<int> dims_ = {7, 7, 7, 1, 2, 2, 1, 1};
        //std::vector<int> times_ = {N_, N_, N_, 1, 1, 1, 1, N_-1};
        std::vector<std::string> var_names_ = {"q_panda", "v_panda", "a_panda", "us", "h"};
        std::vector<int> dims_ = {7, 7, 7, 7, 1};
        std::vector<int> times_ = {N_, N_, N_, N_, N_-1};
        //std::string ref_traj_path_ = "/home/rosdrake/src/src/mpc/traj_refs/1.npy";
        std::string ref_traj_path_ = "/home/rosdrake/src/src/mpc/traj_refs/joint7_traj_N60_tf1.2.npy";
        MyUtils::ProcessedSolution data_proc_; 

        /* Now I learnt, using unique_ptr avoids DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN assertion */
        std::unique_ptr<MPCControllers::LinearMPCProb> prob_;

        std::mutex state_mutex_;

        // Mutex for thread safety
        std::mutex mpc_t_mutex_;
        // std::chrono::time_point<std::chrono::high_resolution_clock> t_start_node_ {};
        double executor_frequency_ {1000.0}; // Frequency of the executor
        drake::trajectories::PiecewisePolynomial<double> u_cmd_spline_ {};
        std::mutex u_cmd_spline_mutex_; // Mutex to protect u_ref_cmd_ access
        Eigen::VectorXd state_now_ {Eigen::VectorXd::Zero(nx_)};
        ros::Timer upsample_timer_;
        bool simulation_ready_signal_ {false}; // Flag to indicate if sim is ready
        std::atomic<bool> received_first_state_ {false}; // Flag to stop the controller thread
        bool mpc_started_ {false}; // Flag to indicate if MPC has started

    };

}