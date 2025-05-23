// #include "linear_mpc_prob.h"
#include <linearmpc_panda/linear_mpc_prob.h>

namespace MPCControllers{

	// constructor
	LinearMPCProb::LinearMPCProb(const std::string& plant_file,
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
				  				 const std::vector<int>& u_entries)
		  : nx_(nx), nu_(nu), h_mpc_(h_mpc), h_env_(h_env), Nt_(Nt), 
		    Nh_(Nt - 1), execution_length_(execution_length), Q_(Q), R_(R), P_(P),
			x_ref_spline_(x_ref_spline), u_ref_spline_(u_ref_spline), integrator_(integrator)
	{ 
		ros::Time t_begin = ros::Time::now();
		ROS_INFO("start at init of linear_mpc_prob.cc! \n");
		
		/*make the plant for the controller with arm only, 
		  no need to weld the finger and set the base pose as only joint space,
		  but need to weld panda base. Otherwise, it is a floating body with 6 extra dofs */
		plant_ptr_ = std::make_unique<MultibodyPlant<double>>(h_env_);
		Parser parser(plant_ptr_.get());
		parser.AddModels(plant_file);
		const auto& arm_base_frame = plant_ptr_->GetFrameByName("panda_link0");
		plant_ptr_->WeldFrames(plant_ptr_->world_frame(), arm_base_frame, X_W_base);

		plant_ptr_->Finalize();
		plantAD_ptr_ = System<double>::ToAutoDiffXd( *(this->plant_ptr_) );
		contextAD_ptr_ = plantAD_ptr_->CreateDefaultContext();
		context_ptr_ = plant_ptr_->CreateDefaultContext();

		//
		udot_up_ = Eigen::VectorXd::Constant(nu_, 1000.0);
		udot_low_ = -udot_up_;

		//initialize the prog
		nDecVar_ = Nh_ * (nx_ + nu_);
		dx_vars_ = prog_.NewContinuousVariables(Nh_, nx_, "joint ds");
		du_vars_ = prog_.NewContinuousVariables(Nh_, nu_, "joint du");

		/* flatten the decision variables */
		MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic> temp(du_vars_.rows(), 
																	du_vars_.cols() + dx_vars_.cols());
		temp << du_vars_, dx_vars_;

		//this create a view, not making a copy
		auto decVar_flat(Eigen::Map<const VectorDecisionVariable<Eigen::Dynamic>>(temp.data(), temp.size()));

		//TO DO: custom cst and costs bindings
		// https://stackoverflow.com/questions/65860331/could-you-demo-a-very-simple-c-example-about-symbolic-variable-and-jacobian-by
		// AN EXAMPLE OF cost and cst bindings
		
		// add cost integral cost
		for (int i = 0; i < Nh_ - 1; ++i)
		{
			auto dx_i = dx_vars_.row(i);
			auto du_i = du_vars_.row(i);

			// fogot that var goes out of scope created in for loop
			auto bx = Eigen::VectorXd::Zero(nx_);
			auto cost = prog_.AddQuadraticCost(Q_, bx, dx_i);
			cost.evaluator()->set_description("Integral cost on s. ");
			auto bu = Eigen::VectorXd::Zero(nu_);
			cost = prog_.AddQuadraticCost(R_, bu, du_i);
			cost.evaluator()->set_description("Integral cost on u. ");
		}

		// add terminal cost
		auto dx_f = dx_vars_.row(Nh_ - 1);
		auto b = Eigen::VectorXd::Zero(nx_);
		/* this will error: prog_.AddCost(0.5 * (dx_f.transpose() * P_ * dx_f)); */
		auto cost = prog_.AddQuadraticCost(P_, b, dx_f); 

		// set up the constraints
		C_cols_ = Nh_ * (nx_ + nu_);

		if (u_entries_.size() == 0)
		{
			C_rows_ = Nh_ * nx_;
		} else {
			C_rows_ = Nh_*nx_ + Nh_*nu_;
		}

		C_ = Eigen::MatrixXd::Zero(C_rows_, C_cols_);
		C_.block(0, nu_, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);
		std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% here checking" << std::endl;

		lb_ = Eigen::VectorXd::Zero(C_rows_);
		ub_ = Eigen::VectorXd::Zero(C_rows_);

		cst_ = prog_.AddLinearConstraint(C_, lb_, ub_, decVar_flat);
		std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% here checking1" << std::endl;

		//initialize solver output
		u_ref_cmd_ = Eigen::MatrixXd::Zero(nu_, Nt_);

		t_init_node_ = ros::Time::now();
		ROS_INFO("LinearMPCProb inited! %f \n", (ros::Time::now() - t_begin).toSec());
	} 

	//######################################################################################
	void LinearMPCProb::f_ad(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y)
	{	
		assert(x.size() == nx_+ nu_ && "xi dim is not the same as nx_!");
		assert((*y).size() == nx_ && "y dim is not the same as nx_!");
		plantAD_ptr_->SetPositionsAndVelocities(contextAD_ptr_.get(), x.segment(0, nx_));
		auto nv = plantAD_ptr_->num_velocities();


		// build the dynamics
		MatrixX<AutoDiffXd> M(nv, nv);
		plantAD_ptr_->CalcMassMatrix(*(contextAD_ptr_.get()), &M);
		auto M_inv = M.inverse();

		VectorX<AutoDiffXd> C(nv);
		plantAD_ptr_->CalcBiasTerm(*(contextAD_ptr_.get()), &C);

		const auto G = plantAD_ptr_->CalcGravityGeneralizedForces(*(contextAD_ptr_.get()));

		auto ddq = M_inv * (G + x.tail(nu_) - C);
		(*y).head(nv) = x.tail(nv);
		(*y).tail(nv) = ddq;
	}

	//######################################################################################
	void LinearMPCProb::f_ad(const Eigen::Ref<const AutoDiffVecXd>& xi,
							 const Eigen::Ref<const AutoDiffVecXd>& ui, 
							 AutoDiffVecXd* yi)
	{	
		//std::cout << plantAD_ptr_->num_velocities() << std::endl;
		assert((*yi).size() == nx_ && "y dim is not the same as nx_!");

		plantAD_ptr_->SetPositionsAndVelocities(contextAD_ptr_.get(), xi);
		auto nv = plantAD_ptr_->num_velocities();

		// build the dynamics
		MatrixX<AutoDiffXd> M(nv, nv);
		plantAD_ptr_->CalcMassMatrix(*(contextAD_ptr_.get()), &M);
		auto M_inv = M.inverse();

		VectorX<AutoDiffXd> C(nv);
		plantAD_ptr_->CalcBiasTerm(*(contextAD_ptr_.get()), &C);
		const auto G = plantAD_ptr_->CalcGravityGeneralizedForces(*(contextAD_ptr_.get()));

		//auto ddq = M_inv * (G + x.tail(nu_) - C);
		auto ddq = M_inv * (G + ui - C);
		//std::cout << "ddq size: " << ddq << std::endl;
		//std::cout << "xi.tail(nv): " << xi.tail(nv).transpose() << std::endl;
		//std::cout << "ddq: " << ddq.transpose() << std::endl;
		(*yi).head(nv) = xi.tail(nv);
		(*yi).tail(nv) = ddq;
	}

	//######################################################################################
	void LinearMPCProb::RK4(const Eigen::Ref<const AutoDiffVecXd>& x, 
							AutoDiffVecXd* y)
	{
		///*Note: xi = [qi; vi; ui] */
		assert(x.size() == nx_+ nu_ && "xi dim is not the same as nx_!");
		assert(y->size() == nx_ && "yi dim is not the same as nx_");

		AutoDiffVecXd temp = x;  //[q, v, u], make a copy as x is const

		//assuming zero order hold on u
		AutoDiffVecXd f1(nx_), f2(nx_), f3(nx_), f4(nx_);
		this->f_ad(temp, 				  &f1);

		temp.head(nx_) = temp.head(nx_) + 0.5*h_mpc_*f1; 
		this->f_ad(temp, &f2);
		
		temp.head(nx_) = temp.head(nx_) + 0.5*h_mpc_*f2; 
		this->f_ad(temp, &f3);

		temp.head(nx_) = temp.head(nx_) + h_mpc_*f3; 
		this->f_ad(temp,     &f4);

		*y = x.segment(0, nx_) + (h_mpc_/6.0) * (f1 + 2*f2 + 2*f3 + f4);
	}

	//######################################################################################
	void LinearMPCProb::RK4(const Eigen::Ref<const AutoDiffVecXd>& xi, 
							const Eigen::Ref<const AutoDiffVecXd>& ui, 
							AutoDiffVecXd* yi)
	{
		/* Note: xi = [qi; vi;] */
		assert(xi.size() == nx_ && "xi dim is not the same as nx_!");
		assert(yi->size() == nx_ && "yi dim is not the same as nx_");

		//assuming zero order hold on u
		AutoDiffVecXd f1(nx_), f2(nx_), f3(nx_), f4(nx_);
		this->f_ad(xi, 				   ui, &f1);
		this->f_ad(xi + 0.5*h_mpc_*f1, ui, &f2);
		this->f_ad(xi + 0.5*h_mpc_*f2, ui, &f3);
		this->f_ad(xi + h_mpc_*f3,     ui, &f4);

		*yi = xi + (h_mpc_/6.0) * (f1 + 2*f2 + 2*f3 + f4);
	}

	//######################################################################################
	void LinearMPCProb::Euler(const Eigen::Ref<const AutoDiffVecXd>& x, AutoDiffVecXd* y)
	{
		/* Note: xi = [qi; vi; ui] */
		assert(x.size() == nx_ && "xi dim is not the same as nx_!");
		assert(y->size() == nx_ && "yi dim is not the same as nx_");

		AutoDiffVecXd temp(nx_);
		this->f_ad(x, &temp);
		*y = x.head(nx_) + h_mpc_ * temp;
	}

	//######################################################################################
	void LinearMPCProb::Euler(const Eigen::Ref<const AutoDiffVecXd>& xi, 
							  const Eigen::Ref<const AutoDiffVecXd>& ui, 
							  AutoDiffVecXd* yi)
	{
		/* Note: xi = [qi; vi;] */
		assert(xi.size() == nx_ && "xi dim is not the same as nx_!");
		assert(yi->size() == nx_ && "yi dim is not the same as nx_");
		AutoDiffVecXd f(nx_); 
		this->f_ad(xi, ui, &f);
		*yi = xi + h_mpc_ * f;
	}

	//######################################################################################
	void LinearMPCProb::Build_C_d_for_solver_errCoord(const std::string& integrator,
													  const Eigen::VectorXd& x0,
													  const Eigen::Ref<Eigen::MatrixXd>& x_ref,
													  const Eigen::Ref<Eigen::MatrixXd>& u_ref,
													  const Eigen::VectorXd dudt_up,
													  const Eigen::VectorXd dudt_low)
	{
		std::cout << "[LinearMPCProb] x0: " << x0.transpose() << std::endl;
		// std::cout << "[LinearMPCProb] u0: " << u_ref.col(0).transpose() << std::endl;

		/* x_ref, u_ref are of Eigen::Matrix<Autodiff, m, n> */
		AutoDiffVecXd fi(nx_);
		auto xi_ad = math::InitializeAutoDiff(x_ref.col(0), nx_+nu_, 0);
		auto ui_ad = math::InitializeAutoDiff(u_ref.col(0), nx_+nu_, nx_); // 3rd arg, grad starting index 
		
		this->RK4(xi_ad, ui_ad, &fi);
		auto fi_grad = math::ExtractGradient(fi);

		assert((fi_grad.cols() == nx_+nu_ && fi_grad.rows() == nx_) && "f0_grad dim is wrong!"); 

		auto A0 = fi_grad.block(0, 0, nx_, nx_);
		//std::cout << "A0: " << A0 << std::endl;
		auto B0 = fi_grad.block(0, nx_, nx_, nu_);
		C_.block(0, 0, nx_, nu_) = B0;

		auto init = -A0 * (x0 - x_ref.col(0));

		//loop to build the lower right part of matrix that corresponds to the dynamics cst
		Eigen::Matrix<double, -1, -1> Ai; /* these 3 are the same: 
		 								     Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
		       							     Eigen::MatrixXd 
										  	 Eigen::Matrix<double, -1, -1>*/
		Eigen::Matrix<double, -1, -1> Bi;
		
		//filling up the big lower right part of the matrix
		for (int i = 0; i < Nh_ - 1; ++i)
		{
			/* for the testing in main.cc the input for the refs are vectors so errors.
			   !! I need to intitialize autodiff inside this function for each grad evaluation.
			      Then I won't be bothered with initialize the refs all together! */
			xi_ad = math::InitializeAutoDiff(x_ref.col(i), nx_+nu_, 0);
			ui_ad = math::InitializeAutoDiff(u_ref.col(i), nx_+nu_, nx_); // 3rd arg, grad starting index 
			this->RK4(xi_ad, ui_ad, &fi);
			fi_grad = math::ExtractGradient(fi);
			Ai = fi_grad.block(0, 0, nx_, nx_);
			Bi = fi_grad.block(0, nx_, nx_, nu_);

			/* In Eigen, the << operator stacks elements row-by-row, not matrices side-by-side. */
			auto C_sub = C_.block((i+1)*nx_, nu_+i*(nx_+nu_), nx_, nx_+nu_+nx_);
			C_sub.leftCols(nx_) = Ai;
			C_sub.middleCols(nx_, nu_) = Bi;
			C_sub.rightCols(nx_) = Eigen::MatrixXd::Identity(nx_, nx_);

		}

		//MyUtils::VisualizeMatSparsity(C_);

		if (u_entries_.size() == 0)
		{
			lb_.head(nx_) = init;
			ub_.head(nx_) = init;
		} else {
			lb_.head(nx_ + nu_) = init;
			lb_.tail(Nh_ * nu_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), udot_low_);
			ub_.head(nx_ + nu_) = init;
			ub_.tail(Nh_ * nu_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), udot_up_);
		}
	}

	//######################################################################################
	void LinearMPCProb::Solve_and_update_C_d_for_solver_errCoord(const Eigen::VectorXd& current_state, 
		  														 ros::Time t_now)
	{
		//query the spline using current time
		//auto ts_mpc = Eigen::VectorXd::LinSpaced(Nt_, t_now, t_now + h_mpc_*Nh_);
		auto ts_mpc = Eigen::VectorXd::LinSpaced(Nt_, (t_now - t_init_node_).toSec(), (t_now - t_init_node_).toSec() + h_mpc_*Nh_);
		std::cout << "ts_mpc: " << ts_mpc.transpose() << std::endl;
		auto x_ref_horizon = x_ref_spline_.vector_values(ts_mpc);
		auto u_ref_horizon = u_ref_spline_.vector_values(ts_mpc);

		assert(x_ref_horizon.cols() == Nt_ && "x_ref_horizon dim is wrong!");
		assert(u_ref_horizon.cols() == Nt_ && "u_ref_horizon dim is wrong!");

		this->Build_C_d_for_solver_errCoord(integrator_, current_state, 
										    x_ref_horizon, u_ref_horizon,
										    udot_up_, udot_low_);

		// evaluator() returns a shared_ptr<LinearConstraint>
		this->cst_->evaluator()->UpdateCoefficients(C_, lb_, ub_); 

		result_ = solver_.Solve(prog_);

		if (result_.is_success())
		{
			std::cout << "Solver success!" << std::endl;
			dx_sol_ = result_.GetSolution(dx_vars_);
			du_sol_ = result_.GetSolution(du_vars_);
			//std::cout << "x_sol: " << x_sol.transpose() << std::endl;
			std::cout << "[LinearMPCProb] du_sol_: " << du_sol_ << std::endl;
		} else {
			std::cout << "Solver failed!" << std::endl;
			std::cout << "Constraint violations: " << std::endl;

		    auto cst_names = result_.GetInfeasibleConstraintNames(prog_);

			for (const auto& name : cst_names) {
				std::cout << name << std::endl;
			}
			return;
		}	

		assert((u_ref_cmd_.rows() == du_sol_.cols() && u_ref_cmd_.cols() == (du_sol_.rows()+1)) 
				&& "u_ref_cmd_ and du_sol_.T dim mismatch!");
		
		u_ref_cmd_.col(0) = u_ref_horizon.col(0);
		//u_ref_cmd of shape: (nu_, Nh_)
		auto temp = u_ref_horizon.block(0, 1, nu_, Nh_);
		//temp.colwise() += xxx // broadcast over all columns
		temp += du_sol_.transpose();
		u_ref_cmd_.block(0, 1, nu_, Nh_) = temp; 

		std::cout << "[LinearMPCProb]_Solve_and_update_C_d_for_solver_errCoord1 u_ref_cmd_: \n" << u_ref_cmd_ << std::endl;
	}

	//######################################################################################
	void LinearMPCProb::Solve_and_update_C_d_for_solver_errCoord(const Eigen::VectorXd& current_state, 
		  														 double t_now_relative) 
    {
		auto ts_mpc = Eigen::VectorXd::LinSpaced(Nt_, t_now_relative, t_now_relative+h_mpc_*Nh_);
		std::cout << "ts_mpc: " << ts_mpc.transpose() << std::endl;
		auto x_ref_horizon = x_ref_spline_.vector_values(ts_mpc);
		auto u_ref_horizon = u_ref_spline_.vector_values(ts_mpc);

		//std::cout << "[LinearMPCProb] x_ref_horizon: " << x_ref_horizon.transpose() << std::endl;
		//std::cout << "[LinearMPCProb] u_ref_horizon: " << u_ref_horizon.transpose() << std::endl;
		std::cout << "[LinearMPCProb] x_ref_horizon at end point: \n" << x_ref_horizon.col(x_ref_horizon.cols()-1).transpose() << std::endl;
		std::cout << "[LinearMPCProb] u_ref_horizon at end point: \n" << u_ref_horizon.col(u_ref_horizon.cols()-1).transpose() << std::endl;

		assert(x_ref_horizon.cols() == Nt_ && "x_ref_horizon dim is wrong!");
		assert(u_ref_horizon.cols() == Nt_ && "u_ref_horizon dim is wrong!");

		this->Build_C_d_for_solver_errCoord(integrator_, current_state, 
										    x_ref_horizon, u_ref_horizon,
										    udot_up_, udot_low_);

		// evaluator() returns a shared_ptr<LinearConstraint>
		this->cst_->evaluator()->UpdateCoefficients(C_, lb_, ub_); 

		result_ = solver_.Solve(prog_);

		if (result_.is_success())
		{
			std::cout << "Solver success!" << std::endl;
			dx_sol_ = result_.GetSolution(dx_vars_);
			du_sol_ = result_.GetSolution(du_vars_);
			//std::cout << "x_sol: " << x_sol.transpose() << std::endl;
			std::cout << "[LinearMPCProb] du_sol_: \n" << du_sol_ << std::endl;
		} else {
			std::cout << "Solver failed!" << std::endl;
			std::cout << "Constraint violations: " << std::endl;

		    auto cst_names = result_.GetInfeasibleConstraintNames(prog_);

			for (const auto& name : cst_names) {
				std::cout << name << std::endl;
			}
			return;
		}	

		assert((u_ref_cmd_.rows() == du_sol_.cols() && u_ref_cmd_.cols() == (du_sol_.rows()+1)) 
				&& "u_ref_cmd_ and du_sol_.T dim mismatch!");
		
		//u_ref_cmd of shape: (nu_, Nh_)
		u_ref_cmd_.col(0) = u_ref_horizon.col(0);
		auto temp = u_ref_horizon.block(0, 1, nu_, Nh_);
		//temp.colwise() += xxx // broadcast over all columns
		std::cout << "temp.shape: " << temp.rows() << " x " << temp.cols() << std::endl;
		std::cout << "du_sol_.shape: " << du_sol_.rows() << " x " << du_sol_.cols() << std::endl;
		temp += du_sol_.transpose();
		std::cout << "check1" << std::endl;
		u_ref_cmd_.block(0, 1, nu_, Nh_) = temp; 
		//u_ref_cmd_spline_ = PiecewisePolynomial<double>::FirstOrderHold(ts_mpc.tail(Nt_-1), u_ref_cmd);

		std::cout << "[LinearMPCProb]_Solve_and_update_C_d_for_solver_errCoord2 u_ref_cmd_: \n" << u_ref_cmd_ << std::endl;
	}
	//######################################################################################
	void LinearMPCProb::Get_solution(Eigen::MatrixXd& output)
	{
		assert((u_ref_cmd_.rows() == output.rows() && u_ref_cmd_.cols() == output.cols()) 
				&&"u_ref_cmd_ and output dim mismatch!");

		output = u_ref_cmd_;
	}

	//######################################################################################
	LinearMPCProb::~LinearMPCProb()
	{
		//std::cout << "Destructor called!" << std::endl;
		plant_ptr_.reset();
		context_ptr_.reset();
	}
	//std::cout << "Destructor called!" << std::endl;
}