#pragma once
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>
#include <iostream>
#include <cassert>
#include <optional>
#include <functional>
#include "myutils.h"

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
					  const std::string& integrator_name,
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
					  const MyUtils::ProcessedSolution& processed_refTraj,
					  //const PiecewisePolynomial<double>& x_ref_spline,
					  //const PiecewisePolynomial<double>& u_ref_spline,
					  const Eigen::VectorXd& u_bounds,
					  const Eigen::VectorXd& x_bounds, 
					  const Eigen::VectorXd& u_entries=Eigen::VectorXd(), // indices for selective constraints on du 
					  const Eigen::VectorXd& x_entries=Eigen::VectorXd()); // indices for selective constraints on dx
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
		void Build_C_d_for_solver_errCoord_RK4( const Eigen::VectorXd& x0,
		 								   const Eigen::Ref<Eigen::MatrixXd>& x_ref,
		 								   const Eigen::Ref<Eigen::MatrixXd>& u_ref,
										   const Eigen::VectorXd dudt_up = {},
										   const Eigen::VectorXd dudt_low = {});
		//
		template <void (LinearMPCProb::*Integrator)(
					const Eigen::Ref<const AutoDiffVecXd>&,
					const Eigen::Ref<const AutoDiffVecXd>&,
					AutoDiffVecXd*)>
		void Build_C_d_for_solver_errCoord(const Eigen::VectorXd& x0,
		 								   const Eigen::Ref<Eigen::MatrixXd>& x_ref,
		 								   const Eigen::Ref<Eigen::MatrixXd>& u_ref,
										   const Eigen::VectorXd dudt_up = {},
										   const Eigen::VectorXd dudt_low = {});

		//
		void Solve_and_update_C_d_for_solver_errCoord(const Eigen::VectorXd& current_state, 
												      double t_now);
		//
		void Solve_and_update_C_d_for_solver_errCoord_test(const Eigen::VectorXd& current_state,
														   const Eigen::MatrixXd& x_ref,
														   const Eigen::MatrixXd& u_ref);

		//
		void Get_solution(Eigen::MatrixXd& output);


	private:
		drake::systems::DiscreteStateIndex state_index_;
		std::string integrator_name_;
		int nx_, nu_, Nt_, Nh_;
		double h_mpc_, h_env_, execution_length_, mpc_horizon_;
		Eigen::MatrixXd Q_;
		Eigen::MatrixXd R_;
		Eigen::MatrixXd P_;
		//Eigen::VectorXd udot_up_ = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
		Eigen::VectorXd x_bounds_;
		Eigen::VectorXd u_bounds_;
		Eigen::VectorXd udot_up_; 
		Eigen::VectorXd udot_low_; 
		AutoDiffVecXd f_grad_;
		MyUtils::ProcessedSolution processed_refTraj_;
		//PiecewisePolynomial<double> x_ref_spline_;
		//PiecewisePolynomial<double> u_ref_spline_;

		Eigen::VectorXd u_entries_;
		Eigen::VectorXd x_entries_;
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
		Eigen::MatrixXd u_ref_cmd_; // for interpolation
		PiecewisePolynomial<double> u_ref_cmd_spline_;
		//VectorDecisionVariable<Eigen::Dynamic> decVar_flat_;
		int C_rows_;
		int C_cols_;
		int lb_dim_;
		Eigen::MatrixXd C_;
		Eigen::VectorXd lb_;
		Eigen::VectorXd ub_;
	};

}