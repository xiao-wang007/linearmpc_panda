#include "linearmpc_panda/linear_mpc_prob.h"

namespace MPCControllers {

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
								 const MyUtils::ProcessedSolution& processed_refTraj,
								 //const PiecewisePolynomial<double>& x_ref_spline,
								 //const PiecewisePolynomial<double>& u_ref_spline,
								 const Eigen::VectorXd& u_up,
								 const Eigen::VectorXd& u_low,
								 const Eigen::VectorXd& x_up,
								 const Eigen::VectorXd& x_low,
				  				 const Eigen::VectorXd& u_entries,
								 const Eigen::VectorXd& x_entries)
		  : nx_(nx), nu_(nu), h_mpc_(h_mpc), h_env_(h_env), Nt_(Nt), 
		    Nh_(Nt - 1), execution_length_(execution_length), Q_(Q), R_(R), P_(P),
			processed_refTraj_(processed_refTraj), integrator_name_(integrator),
			u_entries_(u_entries), x_entries_(x_entries), u_up_(u_up), u_low_(u_low), x_up_(x_up), x_low_(x_low)
	{ 
		std::cout << "checking in linear_mpc_prob.cc constructor 1" << std::endl;
		mpc_horizon_ = h_mpc_ * Nh_;
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

		std::cout << "checking in linear_mpc_prob.cc constructor 2" << std::endl;
		
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
		C_rows_ = Nh_*nx_ + Nh_*nu_ + Nh_*nx_;

		C_ = Eigen::MatrixXd::Zero(C_rows_, C_cols_);
		C_.block(0, nu_, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);

		std::cout << "checking in linear_mpc_prob.cc constructor 3" << std::endl;
		// create matrix to select du & dx
		Eigen::MatrixXd u_block(nu_, nu_ + nx_);
		Eigen::MatrixXd x_block(nx_, nu_ + nx_);
		u_block.setZero();
		x_block.setZero();
		/*.leftCols() return a view, not a resisable object */
		//u_block.leftCols(nu_) = u_entries_.asDiagonal();
		//x_block.leftCols(nx_) = x_entries_.asDiagonal();
		u_block.block(0, 0, nu_, nu_) = u_entries_.asDiagonal();
		x_block.block(0, 0, nx_, nx_) = x_entries_.asDiagonal();
		std::cout << "checking in linear_mpc_prob.cc constructor 4" << std::endl;

		Eigen::MatrixXd u_selected = Eigen::MatrixXd::Zero(Nh_*nu_, Nh_*(nu_+nx_));
		Eigen::MatrixXd x_selected = Eigen::MatrixXd::Zero(Nh_*nx_, Nh_*(nu_+nx_));
		for (int i = 0; i < Nh_; ++i) 
		{
			u_selected.block(i * nu_, i * (nu_ + nx_), nu_, nu_ + nx_) = u_block;
			x_selected.block(i * nx_, i * (nu_ + nx_), nx_, nu_ + nx_) = x_block;
		}

		// put u_selected and x_selected into C_
		C_.block(Nh_*nx_,       0, Nh_*nu_, C_cols_) = u_selected;
		C_.block(Nh_*(nx_+nu_), 0, Nh_*nx_, C_cols_) = x_selected;

		std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% here checking in LinearMPCProb() init" << std::endl;

		lb_ = Eigen::VectorXd::Zero(C_rows_);
		ub_ = Eigen::VectorXd::Zero(C_rows_);

		cst_ = prog_.AddLinearConstraint(C_, lb_, ub_, decVar_flat);
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

		//const auto G = plantAD_ptr_->CalcGravityGeneralizedForces(*(contextAD_ptr_.get()));
		//auto ddq = M_inv * (G + x.tail(nu_) - C);

		auto ddq = M_inv * (x.tail(nu_) - C); // panda has internal compensation
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

		//const auto G = plantAD_ptr_->CalcGravityGeneralizedForces(*(contextAD_ptr_.get()));
		////auto ddq = M_inv * (G + x.tail(nu_) - C);
		//auto ddq = M_inv * (G + ui - C);

		auto ddq = M_inv * (ui - C);
		//std::cout << "ddq size: " << ddq << std::endl;
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
		//std::cout << "hello here 2" << std::endl;
		std::cout << *yi << std::endl;
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
	void LinearMPCProb::Build_C_d_for_solver_errCoord_RK4(const Eigen::VectorXd& x0,
														  const Eigen::Ref<Eigen::MatrixXd>& x_ref,
														  const Eigen::Ref<Eigen::MatrixXd>& u_ref,
														  const Eigen::VectorXd dudt_up,
														  const Eigen::VectorXd dudt_low)
	{
		/* x_ref, u_ref are of Eigen::Matrix<Autodiff, m, n> */
		//Continue here!
		AutoDiffVecXd fi(nx_);
		auto xi_ad = math::InitializeAutoDiff(x_ref.col(0), nx_+nu_, 0);
		auto ui_ad = math::InitializeAutoDiff(u_ref.col(0), nx_+nu_, nx_); // 3rd arg, grad starting index 
		
		this->RK4(xi_ad, ui_ad, &fi);
		auto fi_grad = math::ExtractGradient(fi);

		assert((fi_grad.cols() == nx_+nu_ && fi_grad.rows() == nx_) && "f0_grad dim is wrong!"); 

		auto A0 = fi_grad.block(0, 0, nx_, nx_);
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
	template <void (LinearMPCProb::*Integrator)(
				const Eigen::Ref<const AutoDiffVecXd>&,
				const Eigen::Ref<const AutoDiffVecXd>&,
				AutoDiffVecXd*)>
	void LinearMPCProb::Build_C_d_for_solver_errCoord(const Eigen::VectorXd& x0,
									   const Eigen::Ref<Eigen::MatrixXd>& x_ref,
									   const Eigen::Ref<Eigen::MatrixXd>& u_ref)
	{
		/* x_ref, u_ref are of Eigen::Matrix<Autodiff, m, n> */
		//Continue here!
		AutoDiffVecXd fi(nx_);
		auto xi_ad = math::InitializeAutoDiff(x_ref.col(0), nx_+nu_, 0);
		auto ui_ad = math::InitializeAutoDiff(u_ref.col(0), nx_+nu_, nx_); // 3rd arg, grad starting index 
		
		(this->*Integrator)(xi_ad, ui_ad, &fi);
		auto fi_grad = math::ExtractGradient(fi);

		assert((fi_grad.cols() == nx_+nu_ && fi_grad.rows() == nx_) && "f0_grad dim is wrong!"); 

		auto A0 = fi_grad.block(0, 0, nx_, nx_);
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
			(this->*Integrator)(xi_ad, ui_ad, &fi);
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

		//if (u_entries_.size() == 0)
		//{
			//lb_.head(nx_) = init;
			//ub_.head(nx_) = init;
		//} else {
			//lb_.head(nx_ + nu_) = init;
			//lb_.tail(Nh_ * nu_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), udot_low_);
			//ub_.head(nx_ + nu_) = init;
			//ub_.tail(Nh_ * nu_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), udot_up_);
		//}
		
		lb_.head(nx_) = init;
		ub_.head(nx_) = init;

		////index for du
		//lb_.segment(Nh_*nx_, Nh_*nu_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), );
		//ub_.segment(Nh_*nx_, Nh_*nu_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), );
		//lb_.tail(Nh_ * nx_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), );
		//ub_.tail(Nh_ * nx_) = Eigen::kroneckerProduct(Eigen::MatrixXd::Identity(Nh_, Nh_), );

		// index for du & dx
		for (int i = 0; i < Nh_; i++)
		{
			// for du
			lb_.segment(Nh_*nx_ + i*nu_, nu_) = (u_low_ - u_ref.col(i)) / h_mpc_; // approx for +du/dt
			ub_.segment(Nh_*nx_ + i*nu_, nu_) = (u_up_ - u_ref.col(i)) / h_mpc_; // approx for -du/dt

			// for dx
			lb_.segment(Nh_*(nx_+nu_) + i*nx_, nx_) = (x_low_- x_ref.col(i)) / h_mpc_; // approx for +dx/dt
			ub_.segment(Nh_*(nx_+nu_) + i*nx_, nx_) = (x_up_ - x_ref.col(i)) / h_mpc_; // approx for -dx/dt
		}

	}
	//######################################################################################
	void LinearMPCProb::Solve_and_update_C_d_for_solver_errCoord(const Eigen::VectorXd& current_state, 
																 double t_now)
	{
		//query the spline using current time
		auto ts_mpc = Eigen::VectorXd::LinSpaced(Nt_, t_now, t_now + mpc_horizon_);

 		/* There may be a time offset? can the robot keep up? */
		auto x_ref_horizon = processed_refTraj_.x_ref_spline.vector_values(ts_mpc);
		auto u_ref_horizon = processed_refTraj_.u_ref_spline.vector_values(ts_mpc);

		assert(x_ref_horizon.cols() == Nt_ && "x_ref_horizon dim is wrong!");
		assert(u_ref_horizon.cols() == Nt_ && "u_ref_horizon dim is wrong!");

		if (integrator_name_ == "Euler")
		{
			this->Build_C_d_for_solver_errCoord<&LinearMPCProb::Euler>(current_state, 
																x_ref_horizon, u_ref_horizon);
		} 
		else if (integrator_name_ == "RK4") 
		{
		    this->Build_C_d_for_solver_errCoord<&LinearMPCProb::RK4>(current_state, 
																x_ref_horizon, u_ref_horizon);
		}

		// evaluator() returns a shared_ptr<LinearConstraint>
		this->cst_->evaluator()->UpdateCoefficients(C_, lb_, ub_); 

		result_ = solver_.Solve(prog_);

		if (result_.is_success())
		{
			std::cout << "Solver success!" << std::endl;
			dx_sol_ = result_.GetSolution(dx_vars_);
			du_sol_ = result_.GetSolution(du_vars_);
			//std::cout << "x_sol: " << x_sol.transpose() << std::endl;
			//std::cout << "u_sol: " << u_sol.transpose() << std::endl;
		} else {
			std::cout << "Solver failed!" << std::endl;
			std::cout << "Constraint violations: " << std::endl;

		    auto cst_names = result_.GetInfeasibleConstraintNames(prog_);

			for (const auto& name : cst_names) {
				std::cout << name << std::endl;
			}
			return;
		}	

		u_ref_horizon.block(0, 1, nu_, Nh_) += du_sol_.transpose(); //u_ref_horizon[1:, :]
		u_ref_cmd_ = u_ref_horizon;
		u_ref_cmd_spline_ = PiecewisePolynomial<double>::FirstOrderHold(ts_mpc, u_ref_cmd_);
	}

	//######################################################################################
	void LinearMPCProb::Solve_and_update_C_d_for_solver_errCoord_test(const Eigen::VectorXd& current_state,
	   																  const Eigen::MatrixXd& x_ref,
																	  const Eigen::MatrixXd& u_ref)
	{
		std::cout << "Solve_and_update_C_d_for_solver_errCoord_test called!" << std::endl;
		//query the spline using current time
		assert(x_ref.cols() == Nt_ && "x_ref_horizon dim is wrong!");
		assert(u_ref.cols() == Nt_ && "u_ref_horizon dim is wrong!");

		auto x_ref_horizon = x_ref;
		auto u_ref_horizon = u_ref;

		if (integrator_name_ == "RK4")
		{
			this->Build_C_d_for_solver_errCoord<&LinearMPCProb::RK4>(current_state, 
																	x_ref_horizon, u_ref_horizon);
		} else if (integrator_name_ == "Euler") {
			this->Build_C_d_for_solver_errCoord<&LinearMPCProb::Euler>(current_state, 
																	x_ref_horizon, u_ref_horizon);
		} else {
			std::cout << "Integrator not supported!" << std::endl;
			return;
		}

		// evaluator() returns a shared_ptr<LinearConstraint>
		this->cst_->evaluator()->UpdateCoefficients(C_, lb_, ub_); 

		result_ = solver_.Solve(prog_);

		if (result_.is_success())
		{
			std::cout << "Solver success!" << std::endl;
			dx_sol_ = result_.GetSolution(dx_vars_);
			du_sol_ = result_.GetSolution(du_vars_);
			//std::cout << "x_sol: " << x_sol.transpose() << std::endl;
			//std::cout << "u_sol: " << u_sol.transpose() << std::endl;
		} else {
			std::cout << "Solver failed!" << std::endl;
			std::cout << "Constraint violations: " << std::endl;

		    auto cst_names = result_.GetInfeasibleConstraintNames(prog_);

			for (const auto& name : cst_names) {
				std::cout << name << std::endl;
			}
			return;
		}	
		
		//std::cout << "du_sol_ shape: (" << du_sol_.rows() << ", " << du_sol_.cols() << ")" << std::endl;
		u_ref_horizon.block(0, 1, nu_, Nh_) += du_sol_.transpose(); //u_ref_horizon[1:, :]
		std::cout << "u_ref_horizon shape: (" << u_ref_horizon.rows() << ", " << u_ref_horizon.cols() << ")" << std::endl;
		std::cout << "u_ref_horizon: \n" << u_ref_horizon << '\n' << std::endl;

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