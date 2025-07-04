#include "linearmpc_panda/linear_mpc_prob.h"

namespace MPCControllers {

	// constructor
	LinearMPCProb::LinearMPCProb(const std::string& plant_file,
			                     const std::string& integrator,	
								 bool exclude_gravity,
								 bool decVar_bounds_on,
								 bool udot_bounds_on,
								 int N,
				  				 int nx, 
				  				 int nu, 
				  				 double h_mpc,
				  				 double h_env,
				  				 int Nt,
				  				 RigidTransform<double> X_W_base,
						  		 Eigen::VectorXd Q_diag_vec,
						  		 Eigen::VectorXd R_diag_vec,
								 const MyUtils::ProcessedSolution& processed_refTraj,
								 const Eigen::VectorXd& u_up,
								 const Eigen::VectorXd& u_low,
								 const Eigen::VectorXd& x_up,
								 const Eigen::VectorXd& x_low,
				  				 const Eigen::VectorXd& u_entries,
								 const Eigen::VectorXd& x_entries)
		  : nx_(nx), nu_(nu), h_mpc_(h_mpc), h_env_(h_env), Nt_(Nt), N_(N),
		    Nh_(Nt - 1), Q_diag_vec_(Q_diag_vec), R_diag_vec_(R_diag_vec), 
			processed_refTraj_(processed_refTraj), integrator_name_(integrator), exclude_gravity_(exclude_gravity),
			u_entries_(u_entries), x_entries_(x_entries), u_up_(u_up), u_low_(u_low), x_up_(x_up), x_low_(x_low),
			decVar_bounds_on_(decVar_bounds_on), udot_bounds_on_(udot_bounds_on)
	{ 
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
		udot_up_.resize(nu_);
		udot_up_ = Eigen::VectorXd::Constant(nu_, 1000.0);
		udot_low_.resize(nu_);
		udot_low_ = -udot_up_;

		//initialize the prog
		nDecVar_ = Nh_ * (nx_ + nu_);
		dx_vars_ = prog_.NewContinuousVariables(Nh_, nx_, "joint ds");
		du_vars_ = prog_.NewContinuousVariables(Nh_, nu_, "joint du");

		/* flatten the decision variables */
		MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic> temp(du_vars_.rows(), 
																	du_vars_.cols() + dx_vars_.cols());
		/* flatten the decision variables */
		temp << du_vars_, dx_vars_;

		//this create a view, not making a copy
		auto decVar_flat(Eigen::Map<const VectorDecisionVariable<Eigen::Dynamic>>(temp.data(), temp.size()));

		/* scaling du and dx using: 
		   u_max - u_ref_average and x_max - x_ref_average, using standard deviation (as my trajectory is dynamic)
     	   then I don't have to update then in the MPC loop */
		//compute column-wise mean of the reference trajectory
		Eigen::MatrixXd Q_scaled, R_scaled;
		this->Scale_Q_and_R_by_ref_stddev(Q_scaled, R_scaled);

		//TO DO: custom cst and costs bindings
		// https://stackoverflow.com/questions/65860331/could-you-demo-a-very-simple-c-example-about-symbolic-variable-and-jacobian-by
		// AN EXAMPLE OF cost and cst bindings
		
		// add cost integral cost
		for (int i = 0; i < Nh_ - 1; ++i)
		{
			auto dx_i = dx_vars_.row(i);
			auto du_i = du_vars_.row(i);

			/* scale du and dx in the cost to have similar magnitude using 
			(u_max - u_ref)_max and (x_max - x_ref)_max*/
			auto bx = Eigen::VectorXd::Zero(nx_);
			auto cost = prog_.AddQuadraticCost(Q_scaled, bx, dx_i);
			cost.evaluator()->set_description("Integral cost on s. ");
			auto bu = Eigen::VectorXd::Zero(nu_);
			cost = prog_.AddQuadraticCost(R_scaled, bu, du_i);
			cost.evaluator()->set_description("Integral cost on u. ");
		}

        //make P
        Eigen::MatrixXd Af, Bf;
        Eigen::VectorXd x_f(nx_);
        x_f << processed_refTraj_.trajs.at("q_panda").row(N_-1).transpose(), 
               processed_refTraj_.trajs.at("v_panda").row(N_-1).transpose();
        auto u_f = processed_refTraj_.trajs.at("us").row(N_-1).transpose();
        this->LinearizeAtReference(x_f, u_f, Af, Bf);

		// Q_scaled and R_scaled must be positive semidefinite matrices
		std::cout << "Scaled Q: \n" << Q_scaled << std::endl;
		std::cout << "Scaled R: \n" << R_scaled << std::endl;

		assert(!Q_scaled.hasNaN() && "Q_scaled has NaN values!");
		DRAKE_DEMAND(Q_scaled.allFinite() && "Q_scaled must be finite!");
		assert(!R_scaled.hasNaN() && "R_scaled has NaN values!");
		DRAKE_DEMAND(R_scaled.allFinite() && "R_scaled must be finite!");

        LinearQuadraticRegulatorResult K_and_S = DiscreteTimeLinearQuadraticRegulator(Af, Bf, Q_scaled, R_scaled);
        P_ = K_and_S.S; // P is the solution to the discrete-time algebraic Riccati equation (DARE)
		assert(!P_.hasNaN() && "P_ has NaN values!");
        //P_ = Eigen::MatrixXd::Identity(nx_, nx_) * 100;

		// add terminal cost
		auto dx_f = dx_vars_.row(Nh_ - 1);
		auto b = Eigen::VectorXd::Zero(nx_);
		/* this will error: prog_.AddCost(0.5 * (dx_f.transpose() * P_ * dx_f)); */
		auto cost = prog_.AddQuadraticCost(P_, b, dx_f); 

		// set up the constraints
		C_cols_ = Nh_ * (nx_ + nu_);

		/* correspond to: 
		   dynamics constraints,
		   du bounds,
		   dx bounds,
		   dtau bounds*/ 
		if (decVar_bounds_on_)
		{
			C_rows_ = Nh_*nx_ + Nh_*nu_ + Nh_*nx_ + (Nh_-1)*nu_;
			C_ = Eigen::MatrixXd::Zero(C_rows_, C_cols_);

			// call member function to populate C_ for selecting decision variables
			this->Populate_C_for_selecting_decision_variables(x_entries_, u_entries_);

			// call member function to populate C_ for selecting du for dtau
			this->Populate_C_for_selecting_du_for_dtau();

		} else if (udot_bounds_on_){
			C_rows_ = Nh_*nx_ + (Nh_-1)*nu_;
			C_ = Eigen::MatrixXd::Zero(C_rows_, C_cols_);
			this->Populate_C_for_selecting_du_for_dtau();
		}

		C_.block(0, nu_, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);

		lb_ = Eigen::VectorXd::Zero(C_rows_);
		ub_ = Eigen::VectorXd::Zero(C_rows_);
		cst_ = prog_.AddLinearConstraint(C_, lb_, ub_, decVar_flat);

		std::cout << "######################################## LinearMPCProb() inited!" << std::endl;
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

		VectorX<AutoDiffXd> ddq(nv);
		if (exclude_gravity_)
		{
			ddq = M_inv * (x.tail(nu_) - C); // panda has internal compensation
		}
		else {
			const auto G = plantAD_ptr_->CalcGravityGeneralizedForces(*(contextAD_ptr_.get()));
			ddq = M_inv * (G + x.tail(nu_) - C);
		}

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

		VectorX<AutoDiffXd> ddq(nv);
		if (exclude_gravity_)
		{
			ddq = M_inv * (ui - C);
		} else {
			const auto G = plantAD_ptr_->CalcGravityGeneralizedForces(*(contextAD_ptr_.get()));
			ddq = M_inv * (G + ui - C);
		}

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
			/* // Correct! 2nd arg in .block(), nu_ offsets B0
			int starting_row = (i+1)*nx_;
			int starting_col = nu_+i*(nx_+nu_);
			auto C_sub = C_.block(starting_row, starting_col, nx_, nx_+nu_+nx_);
			C_sub.leftCols(nx_) = Ai;
			C_sub.middleCols(nx_, nu_) = Bi;
			C_sub.rightCols(nx_) = Eigen::MatrixXd::Identity(nx_, nx_);
			*/

			// a more efficient way
			C_.block((i+1)*nx_, nu_+i*(nx_+nu_), nx_, nx_) = Ai;
			C_.block((i+1)*nx_, nu_+i*(nx_+nu_) + nx_, nx_, nu_) = Bi;
			C_.block((i+1)*nx_, nu_+i*(nx_+nu_) + nx_ + nu_, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);
		}

		//MyUtils::VisualizeMatSparsity(C_);

		/* the bounds on decision variables change as well, so I have to do it here. */
		lb_.head(nx_) = init;
		ub_.head(nx_) = init;

		/*
		// index for du & dx
		for (int i = 0; i < Nh_; i++)
		{
			// bounds for du
			lb_.segment(Nh_*nx_ + i*nu_, nu_) = (u_low_ - u_ref.col(i)); 
			ub_.segment(Nh_*nx_ + i*nu_, nu_) = (u_up_ - u_ref.col(i)); 

			// bounds for dx
			lb_.segment(Nh_*(nx_+nu_) + i*nx_, nx_) = (x_low_- x_ref.col(i)); 
			ub_.segment(Nh_*(nx_+nu_) + i*nx_, nx_) = (x_up_ - x_ref.col(i)); 
		} */

		if (decVar_bounds_on_)
		{
			/* using kroneckerProduct to do the same */ 
			Eigen::VectorXd u_up_full = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(Nh_), u_up_);
			auto u_low_full = -u_up_full;

			Eigen::VectorXd x_up_full = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(Nh_), x_up_);
			Eigen::VectorXd x_low_full = Eigen::kroneckerProduct(Eigen::VectorXd::Ones(Nh_), x_low_);

			// x_ref_flat is a new copy of column-wise concatenation of x_ref
			//Eigen::VectorXd x_ref_flat = Eigen::Map<const Eigen::VectorXd>(x_ref.block(0, 1, x_ref.rows(), x_ref.cols() - 1).data(), Nh_ * nx_);
			//Eigen::VectorXd u_ref_flat = Eigen::Map<const Eigen::VectorXd>(u_ref.block(0, 0, u_ref.rows(), u_ref.cols() - 1).data(), Nh_ * nu_);
			Eigen::VectorXd x_ref_flat = Eigen::Map<const Eigen::VectorXd>(x_ref.data(), Nh_ * nx_);
			Eigen::VectorXd u_ref_flat = Eigen::Map<const Eigen::VectorXd>(u_ref.data(), Nh_ * nu_);

			//std::cout << "u_low_full shape: " << u_low_full.rows() << " x " << u_low_full.cols() << std::endl; 
			//std::cout << "u_ref_flat shape: " << u_ref_flat.rows() << " x " << u_ref_flat.cols() << std::endl;

			assert(u_low_full.size() == u_ref_flat.size() && "u_low_full size is not the same as u_ref_flat size!");
			assert(x_low_full.size() == x_ref_flat.size() && "x_low_full size is not the same as x_ref_flat size!");

			// set bounds for du 
			lb_.segment(Nh_*nx_, Nh_*nu_) = u_low_full - u_ref_flat;
			ub_.segment(Nh_*nx_, Nh_*nu_) = u_up_full - u_ref_flat;

			// set bounds for dx
			lb_.segment(Nh_*(nx_+nu_), Nh_*nx_) = x_low_full - x_ref_flat;
			ub_.segment(Nh_*(nx_+nu_), Nh_*nx_) = x_up_full - x_ref_flat;

			// set bounds for dtau constraint
			// this does [u1, u2, u3] - [u0, u1, u2]
			Eigen::MatrixXd diff = u_ref.block(0, 1, u_ref.rows(), u_ref.cols() - 1) - u_ref.block(0, 0, u_ref.rows(), u_ref.cols() - 1);
			Eigen::VectorXd diff_flat = Eigen::Map<const Eigen::VectorXd>(diff.data(), diff.size());

			lb_.segment(Nh_*(nx_ + nu_ + nx_), (Nh_-1)*nu_) = Eigen::VectorXd::Constant(diff_flat.size(), -1000.*h_mpc_) - diff_flat;
			ub_.segment(Nh_*(nx_ + nu_ + nx_), (Nh_-1)*nu_) = Eigen::VectorXd::Constant(diff_flat.size(),  1000.*h_mpc_) - diff_flat;

		} else if (udot_bounds_on_) {
			// set bounds for dtau constraint
			// this does [u1, u2, u3] - [u0, u1, u2]
			Eigen::MatrixXd diff = u_ref.block(0, 1, u_ref.rows(), u_ref.cols() - 1) - u_ref.block(0, 0, u_ref.rows(), u_ref.cols() - 1);
			Eigen::VectorXd diff_flat = Eigen::Map<const Eigen::VectorXd>(diff.data(), diff.size());

			lb_.segment(Nh_*nx_, (Nh_-1)*nu_) = Eigen::VectorXd::Constant(diff_flat.size(), -1000.*h_mpc_) - diff_flat;
			ub_.segment(Nh_*nx_, (Nh_-1)*nu_) = Eigen::VectorXd::Constant(diff_flat.size(),  1000.*h_mpc_) - diff_flat;
		}
	}

	//######################################################################################
	void LinearMPCProb::Populate_C_for_selecting_decision_variables(const Eigen::VectorXd& x_entries,
																	const Eigen::VectorXd& u_entries)
	{
		// check C_ is initialized
        assert(C_.rows() > 0 && C_.cols() > 0 && "C_ is not initialized!");

		/*
		// select du 
		for (int i = 0; i < Nh_; i++)
		{
			// Nh_*nx_ skips the rows for dynamics constraint
			int start_row = Nh_ * nx_ + i * nu_;
			int start_col = i * (nu_ + nx_);
			C_.block(start_row, start_col, nu_, nu_) = u_entries.asDiagonal();
		}

		// select dx
		for (int i = 0; i < Nh_; i++)
		{
			// Nh_*(nx_+nu_) skips the rows for dynamics constraint and the selection of du
		    int start_row = Nh_ * (nx_ + nu_) + i * nx_;
			int start_col = i * (nu_ + nx_) + nu_;
			C_.block(start_row, start_col, nx_, nx_) = x_entries.asDiagonal();
		}*/

		int reps = Nh_;
		Eigen::MatrixXd I_rep = Eigen::MatrixXd::Identity(reps, reps);
		Eigen::MatrixXd block_du = u_entries.asDiagonal();
		Eigen::MatrixXd block_dx = x_entries.asDiagonal();

		int du_starting_row = Nh_ * nx_;
		C_.block(du_starting_row, 0, nu_*reps, C_.cols()) = Eigen::kroneckerProduct(I_rep, block_du);

		int dx_starting_row = Nh_ * (nx_ + nu_);
		C_.block(dx_starting_row, 0, nx_*reps, C_.cols()) = Eigen::kroneckerProduct(I_rep, block_dx);
	}

	//######################################################################################
	void LinearMPCProb::Populate_C_for_selecting_du_for_dtau()
	{
		// check C_ is initialized
		assert(C_.rows() > 0 && C_.cols() > 0 && "C_ is not initialized!");
		int reps = Nh_ - 1;

		Eigen::MatrixXd I_rep = Eigen::MatrixXd::Identity(reps, reps);
		Eigen::MatrixXd block = Eigen::MatrixXd::Zero(nu_, nu_ + nx_ + nu_);

		block << -Eigen::MatrixXd::Identity(nu_, nu_), 
				  Eigen::MatrixXd::Zero(nu_, nx_), 
				  Eigen::MatrixXd::Identity(nu_, nu_);

        assert (C_.cols() == Nh_ * (nx_ + nu_) && "C_ cols is not correct!");

		if (decVar_bounds_on_)
		{
			C_.block(Nh_*(nx_ + nu_ + nx_), 0, nu_*reps, C_.cols()) = Eigen::kroneckerProduct(I_rep, block);
		} else if (udot_bounds_on_) {
			C_.block(Nh_*nx_, 0, nu_*reps, C_.cols()) = Eigen::kroneckerProduct(I_rep, block);
		}
	}


	//######################################################################################
	void LinearMPCProb::Scale_Q_and_R_by_ref_stddev(Eigen::MatrixXd& Q_scaled, Eigen::MatrixXd& R_scaled)
	{
		//Eigen::RowVectorXd du_average = (processed_refTraj_.trajs.at("us").rowwise() - u_up_.transpose()).colwise().mean();
		//Eigen::RowVectorXd dq_average = (processed_refTraj_.trajs.at("q_panda").rowwise() - x_up_.transpose().head(nu_)).colwise().mean();
		//Eigen::RowVectorXd dv_average = (processed_refTraj_.trajs.at("v_panda").rowwise() - x_up_.transpose().tail(nu_)).colwise().mean();

		Eigen::RowVectorXd du_average = (u_up_.transpose().replicate(processed_refTraj_.trajs.at("us").rows(), 1) - processed_refTraj_.trajs.at("us")).colwise().mean();
		Eigen::RowVectorXd dq_average = (x_up_.transpose().head(nu_).replicate(processed_refTraj_.trajs.at("q_panda").rows(), 1) - processed_refTraj_.trajs.at("q_panda")).colwise().mean();
		Eigen::RowVectorXd dv_average = (x_up_.transpose().tail(nu_).replicate(processed_refTraj_.trajs.at("v_panda").rows(), 1) - processed_refTraj_.trajs.at("v_panda")).colwise().mean();

		std::cout << "u_ref_average: " << du_average << std::endl;
		std::cout << "q_ref_average: " << dq_average << std::endl;
		std::cout << "v_ref_average: " << dv_average << std::endl;
		std::cout << '\n' << std::endl;

		Eigen::RowVectorXd u_average_squared_error = du_average.array().square();
		Eigen::RowVectorXd x_average_squared_error(nx_);
		x_average_squared_error << dq_average.array().square(), dv_average.array().square();

		Q_scaled = (Q_diag_vec_.transpose().array() * (1. / x_average_squared_error.array())).matrix().asDiagonal();
		R_scaled = (R_diag_vec_.transpose().array() * (1. / u_average_squared_error.array())).matrix().asDiagonal();

		std::cout << "Scaled Q: \n" << Q_scaled << std::endl;
		std::cout << "Scaled R: \n" << R_scaled << std::endl;

		//// compute centred reference trajectories
		//Eigen::MatrixXd u_ref_centred = processed_refTraj_.trajs.at("us").rowwise() - u_ref_average;
		//Eigen::MatrixXd q_ref_centred = processed_refTraj_.trajs.at("q_panda").rowwise() - q_ref_average;
		//Eigen::MatrixXd v_ref_centred = processed_refTraj_.trajs.at("v_panda").rowwise() - v_ref_average;

		//std::cout << "u_ref_centred: \n" << u_ref_centred << std::endl;
		//std::cout << "q_ref_centred: \n" << q_ref_centred << std::endl;
		//std::cout << "v_ref_centred: \n" << v_ref_centred << std::endl;
		//std::cout << '\n' << std::endl;

		//// compute standard deviation of the reference trajectories
		//Eigen::RowVectorXd u_ref_stddev = (u_ref_centred.array().square().colwise().sum() / (processed_refTraj_.trajs.at("us").rows() - 1)).sqrt();
		//Eigen::RowVectorXd q_ref_stddev = (q_ref_centred.array().square().colwise().sum() / (processed_refTraj_.trajs.at("q_panda").rows() - 1)).sqrt();
		//Eigen::RowVectorXd v_ref_stddev = (v_ref_centred.array().square().colwise().sum() / (processed_refTraj_.trajs.at("v_panda").rows() - 1)).sqrt();
		//Eigen::RowVectorXd x_ref_stddev(nx_);
		//x_ref_stddev << q_ref_stddev, v_ref_stddev;

		//std::cout << "u_ref_stddev: \n" << u_ref_stddev << std::endl;
		//std::cout << "x_ref_stddev: \n" << x_ref_stddev << std::endl;
		//std::cout << '\n' << std::endl;

		//Eigen::VectorXd x_stddev_squared = x_ref_stddev.array().square().transpose();
		//Eigen::VectorXd Q_diag_scaled = Q_diag_vec_.array() / x_stddev_squared.array(); // Q/stddev^2
		//Q_scaled = Q_diag_scaled.asDiagonal();

		//Eigen::VectorXd u_stddev_squared = u_ref_stddev.array().square().transpose();
		//Eigen::VectorXd R_diag_scaled = R_diag_vec_.array() / u_stddev_squared.array(); // R/stddev^2
		//R_scaled = R_diag_scaled.asDiagonal();
	}


	//######################################################################################
	void LinearMPCProb::Solve_and_update_C_d_for_solver_errCoord(const Eigen::VectorXd& current_state, 
																 double t_now)
	{
 		/* This is IMPORTANT!!!!!
		   du_vars = [u0, u1, u2, u3] 
		   dx_vars = [x1, x2, x3, x4] 
		   but correspondingly, to compute Ai and Bi, along the horizon, 
		   need [x_ref0, x_ref1, x_ref2, x_ref3] 
		   and  [u_ref0, u_ref1, u_ref2, u_ref3] 
		   Therefore, querying the ref spline, using Nh_ time points 
		   ----------------------------------------------------------
		   | x0  | x1 | x2 | x3 | x4  |  -----> looking forward 4 intervals, Nh_ = 4 = n_xdecVar, Nt_ = 5
		   	  \    \    \    \    /
		   | u0  | u1 | u2 | u3 |      -----> applying 4 control inputs, starting from x0, also Nh_ = 4 = n_udecVar 
		   ----------------------------------------------------------
		   */
		//query the spline using current time
		auto ts_mpc = Eigen::VectorXd::LinSpaced(Nh_, t_now, t_now + mpc_horizon_);

		auto x_ref_horizon = processed_refTraj_.x_ref_spline.vector_values(ts_mpc);
		auto u_ref_horizon = processed_refTraj_.u_ref_spline.vector_values(ts_mpc);

		assert(x_ref_horizon.cols() == Nh_ && "x_ref_horizon dim is wrong!");
		assert(u_ref_horizon.cols() == Nh_ && "u_ref_horizon dim is wrong!");

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

		//assert to debug for nans in QP 
		assert(!C_.hasNaN() && "C_ has NaN values!");
		assert(!lb_.hasNaN() && "lb_ has NaN values!");
		assert(!ub_.hasNaN() && "ub_ has NaN values!");

		// evaluator() returns a shared_ptr<LinearConstraint>
		this->cst_->evaluator()->UpdateCoefficients(C_, lb_, ub_); 

		result_ = solver_.Solve(prog_);

		if (result_.is_success())
		{
			std::cout << "Solver success!" << std::endl;
			dx_sol_ = result_.GetSolution(dx_vars_);
			du_sol_ = result_.GetSolution(du_vars_);
			std::cout << "OSQP Solve time: " 
						   << result_.get_solver_details<OsqpSolver>().solve_time << std::endl;
			//std::cout << "x_sol: " << x_sol.transpose() << std::endl;
			//std::cout << "u_sol: " << u_sol.transpose() << std::endl;
		} else {
			std::cout << "Solver failed!" << std::endl;
			std::cout << "Constraint violations: " << std::endl;

			if (!result_.is_success()) {
			   std::cout << "Solver failed!" << std::endl;

			   // Print the solver status (e.g., "Infeasible", "Max iterations reached")
			   std::cout << "Solver status: " 
						   << result_.get_solver_details<OsqpSolver>().status_val << std::endl;

			   // Print the result's solution status (e.g., kInfeasibleConstraints, kIterationLimit)
			   std::cout << "Solution result: " << result_.get_solution_result() << std::endl;

			   // (Optional) Print the cost at the failed solution
			   std::cout << "Cost at solution: " << result_.get_optimal_cost() << std::endl;
}

		    auto cst_names = result_.GetInfeasibleConstraintNames(prog_);

			for (const auto& name : cst_names) {
				std::cout << name << std::endl;
			}
			return;
		}	

		//u_ref_horizon.block(0, 1, nu_, Nh_) += du_sol_.transpose(); //u_ref_horizon[1:, :]
		u_ref_horizon += du_sol_.transpose(); //u_ref_horizon[1:, :]
		u_ref_cmd_ = u_ref_horizon;
		//u_ref_cmd_spline_ = PiecewisePolynomial<double>::FirstOrderHold(ts_mpc, u_ref_cmd_);
	}

	//######################################################################################
	void LinearMPCProb::Solve_and_update_C_d_for_solver_errCoord_test(const Eigen::VectorXd& current_state,
	   																  const Eigen::MatrixXd& x_ref,
																	  const Eigen::MatrixXd& u_ref)
	{
		std::cout << "Solve_and_update_C_d_for_solver_errCoord_test called!" << std::endl;
		//query the spline using current time
		assert(x_ref.cols() == Nh_ && "x_ref_horizon dim is wrong!");
		assert(u_ref.cols() == Nh_ && "u_ref_horizon dim is wrong!");

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
		//u_ref_horizon.block(0, 1, nu_, Nh_) += du_sol_.transpose(); //u_ref_horizon[1:, :]
		u_ref_horizon += du_sol_.transpose(); //u_ref_horizon[1:, :]
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

    //###############################################################################
    void LinearMPCProb::LinearizeAtReference(const Eigen::VectorXd& x_ref,
                                             const Eigen::VectorXd& u_ref,
                                             Eigen::MatrixXd& A,
                                             Eigen::MatrixXd& B)
    {
        AutoDiffVecXd f_N(nx_);
        auto xN_ad = math::InitializeAutoDiff(x_ref, nx_+nu_, 0);
        auto uN_ad = math::InitializeAutoDiff(u_ref, nx_+nu_, nx_); // 3rd arg, grad starting index 

        if (integrator_name_ == "RK4")
        {
            this->RK4(xN_ad, uN_ad, &f_N);
        } else {
            this->Euler(xN_ad, uN_ad, &f_N);
        }

        auto fN_grad = math::ExtractGradient(f_N);

        assert((fN_grad.cols() == nx_+nu_ && fN_grad.rows() == nx_) && "fN_grad dim is wrong!");
        A = fN_grad.block(0, 0, nx_, nx_);
        B = fN_grad.block(0, nx_, nx_, nu_);
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