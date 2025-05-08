#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>

// from my linermpc in drake c++
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <drake/solvers/osqp_solver.h>
#include <drake/solvers/solve.h>
#include <drake/common/drake_assert.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/math/autodiff.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

namespace linearmpc_panda {
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

    class QPController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            franka_hw::FrankaStateInterface,
            hardware_interface::EffortJointInterface> {
    public:
        /* I should initialize my prog, ref_trajs, etc*/
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

        void go_to_init_pose(const Eigen::VectorXd& init_joint_pos);

        void init_plant();

        void init_prog();

    private:
//        void desiredStateCallback(const panda_inverse_dynamics_controller::DesiredState::ConstPtr& msg);

        ros::Subscriber state_subscriber_;

        std::unique_ptr <franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr <franka_hw::FrankaStateHandle> state_handle_;

        std::vector<hardware_interface::JointHandle> joint_handles_;

		//drake::systems::DiscreteStateIndex state_index_;
        std::string panda_file_ {"/home/rosdrake/panda_arm.urdf"};
		std::string integrator_;
		int nx_, nu_, Nt_, Nh_;
		double h_mpc_, h_env_, execution_length_;
		Eigen::MatrixXd Q_;
		Eigen::MatrixXd R_;
		Eigen::MatrixXd P_;
		//Eigen::VectorXd udot_up_ = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
		Eigen::VectorXd udot_up_; 
		Eigen::VectorXd udot_low_; 
		AutoDiffVecXd f_grad_;
		PiecewisePolynomial<double> x_ref_spline_;
		PiecewisePolynomial<double> u_ref_spline_;

		std::vector<int> u_entries_;
		std::unique_ptr<MultibodyPlant<double>> plant_ptr_;
		std::unique_ptr<Context<double>> context_ptr_;
		std::unique_ptr<MultibodyPlant<AutoDiffXd>> plantAD_ptr_;
		std::unique_ptr<Context<AutoDiffXd>> contextAD_ptr_;
		
		//define the prog
		MathematicalProgram prog_;
		OsqpSolver solver_;
		int nDecVar_;
		MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic> dx_vars_;
		MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic> du_vars_;
		std::optional<solvers::Binding<drake::solvers::LinearConstraint>> cst_;
		solvers::MathematicalProgramResult result_;
		Eigen::MatrixXd dx_sol_;
		Eigen::MatrixXd du_sol_;
		Eigen::MatrixXd u_ref_cmd; // for interpolation
		PiecewisePolynomial<double> u_ref_cmd_spline_;
		//VectorDecisionVariable<Eigen::Dynamic> decVar_flat_;
		int C_rows_;
		int C_cols_;
		int lb_dim_;
		Eigen::MatrixXd C_;
		Eigen::VectorXd lb_;
		Eigen::VectorXd ub_;


    };
} // namespace linearmpc_panda