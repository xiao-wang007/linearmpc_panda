#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <iostream>
#include <cassert>
#include <optional>
#include "myutils.h"
#include <ros/ros.h>

#include <drake/solvers/osqp_solver.h>
#include <drake/solvers/solve.h>
#include <drake/common/drake_assert.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/math/autodiff.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

namespace MPCControllers{
	//
	using drake::multibody::MultibodyPlant;
	using drake::multibody::Parser;
	using drake::systems::Context;
	using drake::systems::System;
	using drake::AutoDiffXd;
	using drake::math::InitializeAutoDiff;
	using drake::math::RigidTransform;
	using drake::trajectories::PiecewisePolynomial;

	using namespace drake; // for all eigen types
	using namespace drake::solvers;

	//
	class LinearMPCProb 
	{
	public:
		// have to create the plant here since DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MultibodyPlant)
		LinearMPCProb(const std::string& plant_file,
					  const std::string& integrator,
					  int nx, 
					  int nu, 
					  double execution_length,
					  double h_mpc,
					  double h_env,
					  int Nt,
					  RigidTransform<double> X_W_base,
					  Eigen::MatrixXd Q,
					  Eigen::MatrixXd R,
					  Eigen::MatrixXd P,
					  const PiecewisePolynomial<double>& x_ref_spline,
					  const PiecewisePolynomial<double>& u_ref_spline,
					  const std::vector<int>& u_entries={});
	    ~LinearMPCProb();
		 
		//In Eigen, when you want to pass an Eigen vector by reference (to avoid copying) 
		//while also allowing the function to accept different types of Eigen expressions
		//(like subvectors, matrix blocks, or mapped data), you can use Eigen::Ref.
		void f_ad(const Eigen::Ref<const AutoDiffVecXd>& xi,
				  const Eigen::Ref<const AutoDiffVecXd>& ui, 
				  AutoDiffVecXd* yi);
		void f_ad(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y);

		//
		void RK4(const Eigen::Ref<const AutoDiffVecXd>& xi, 
				 const Eigen::Ref<const AutoDiffVecXd>& ui, 
				 AutoDiffVecXd* yi);
		void RK4(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y);
		
		//
		void Euler(const Eigen::Ref<const AutoDiffVecXd>& xi, 
				   const Eigen::Ref<const AutoDiffVecXd>& ui, 
				   AutoDiffVecXd* yi);
		void Euler(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y);
		
		//
		void Build_C_d_for_solver_errCoord(const std::string& integrator,
		 								   const Eigen::VectorXd& x0,
		 								   const Eigen::Ref<Eigen::MatrixXd>& x_ref,
		 								   const Eigen::Ref<Eigen::MatrixXd>& u_ref,
										   //const Eigen::Ref<const AutoDiffVecXd>& x_ref_ad,
										   //const Eigen::Ref<const AutoDiffVecXd>& u_ref_ad,
										   const Eigen::VectorXd dudt_up = {},
										   const Eigen::VectorXd dudt_low = {});

		//
		void Solve_and_update_C_d_for_solver_errCoord(const Eigen::VectorXd& current_state, 
													  double t_now);

		//
		void Get_solution(Eigen::MatrixXd& output);


	private:
		//drake::systems::DiscreteStateIndex state_index_;
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
		//VectorDecisionVariable<Eigen::Dynamic> decVar_flat_;
		int C_rows_;
		int C_cols_;
		int lb_dim_;
		Eigen::MatrixXd C_;
		Eigen::VectorXd lb_;
		Eigen::VectorXd ub_;

		//solver output
		Eigen::MatrixXd u_ref_cmd_;

		//ros::time
		ros::Time t_init_node_ {};
	};

}