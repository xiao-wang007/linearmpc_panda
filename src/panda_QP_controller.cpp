#include "panda_QP_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/hardware_interface.h>

namespace linearmpc_panda {

	//#######################################################################################
    bool QPController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) {
        //Get Franka model and state interfaces
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        auto *state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        auto *effort_interface = robot_hw->get<hardware_interface::EffortJointInterface>();

        if (!model_interface || !state_interface || !effort_interface) {
            ROS_ERROR("Failed to get required hardware interfaces.");
            return false;
        }

        // Get handles
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle("panda_model"));
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle("panda_robot"));

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();

        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint1"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint2"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint3"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint4"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint5"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint6"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint7"));

		/* Direct access to panda's current state, no need to use a sub, but here in gazebo, I need to do this through ros */
		//get panda state, topic belongs to franka_gazebo, which operates as 1kHz, then the callback is also called at 1kHz
		state_sub_ = node_handle.subscribe("joint_states", 1, &QPController::joint_state_callback_sim, this);

		//get interpolated solution trajectory 
		executor_sub_ = node_handle.subscribe("executor", 1, &QPController::executor_callback, this);

		// compute mpc related parameters, this has to go first as init_prog() uses them
		Nh_ = Nt_ - 1;
		execution_length_ = h_mpc_ * n_exe_steps_;
		mpc_horizon_ = h_mpc_ * Nh_;

        // init prog
        //this->init_prog();

        // init the plant   
        //this->init_plant();

        // load x_ref and u_ref
		data_proc_ = MyUtils::ProcessSolTraj(ref_traj_path_ , var_names_, dims_, times_);

		//init MyControllers::LinearMPCProb() here
		integrator_ = "RK4";
        X_W_base_ = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * PI / 180.),
                                      Vector3<double>(0., -0.2, 0.));

		//make Q
		Eigen::VectorXd q_coef = Eigen::VectorXd::Constant(nu_, 200.0) * 12.;
		Eigen::VectorXd v_coef = Eigen::VectorXd::Constant(nu_, 1.) * 0.1;
		Eigen::VectorXd Q_diags(q_coef.rows() + v_coef.rows());
		Q_diags.head(q_coef.rows()) = q_coef;
		Q_diags.tail(v_coef.rows()) = v_coef;
		Eigen::DiagonalMatrix<double, NUM_JOINTS*2> Q_sparse = Q_diags.asDiagonal();
		Eigen::MatrixXd Q = Q_sparse.toDenseMatrix();

		//make R
		Eigen::VectorXd u_coef = Eigen::VectorXd::Constant(nu_, 0.001);
		Eigen::MatrixXd R = u_coef.asDiagonal();

		//make P
		Eigen::MatrixXd P = Eigen::MatrixXd::Identity(nx_, nx_) * 100;


		/* have to do this to avoid DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN() assertion as 
		   I have a MultibodyPlant() inside LinearMPCProb()*/
		prob_ = std::make_unique<MyControllers::LinearMPCProb>(panda_file_, integrator_, nx_, nu_, execution_length_, 
											 h_mpc_, h_env_, Nt_, X_W_base_, Q, R, P, 
											 data_proc_.x_ref_spline, 
											 data_proc_.u_ref_spline); 

        return true;
    }

	//#######################################################################################
    void QPController::starting(const ros::Time& time) 
    {
        // index the first horizon
		auto t0 = ros::Time::now().toSec();
		xref_now_ = data_proc_.x_ref_spline.vector_values(Eigen::VectorXd::LinSpaced(Nt_, t0, t0+mpc_horizon_));
		uref_now_ = data_proc_.u_ref_spline.vector_values(Eigen::VectorXd::LinSpaced(Nt_, t0, t0+mpc_horizon_));

        // set the condition here to check if current robot state is close to x0_ref

		if (!initial_pose_ok())
		{
			//go_to_init_pose(xref_now_.col(0));
		}
    }

	//#######################################################################################
    void QPController::update(const ros::Time& time, const ros::Duration& period) 
	{
		//get current reference trajectory 
		t_now_ = ros::Time::now().toSec();
		ts_ = Eigen::VectorXd::LinSpaced(Nt_, t_now_, t_now_+mpc_horizon_);
		xref_now_ = data_proc_.x_ref_spline.vector_values(ts_);
		uref_now_ = data_proc_.u_ref_spline.vector_values(ts_);

		/* TODO: check flag if sim or real, then get the states of panda accordingly*/

		//direct access to get current state, panda hardware interface
		const std::array<double, 7>& q_now = state_handle_->getRobotState().q;
		const auto& v_now = state_handle_->getRobotState().dq;
		const auto& tau_now = state_handle_->getRobotState().tau_J;

		// get state_now directly from hardware if not in simulation
		if (!do_sim_)
		{
			robot_state_ = state_handle_->getRobotState();
			q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(robot_state_.q.data());
			v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(robot_state_.dq.data());
			//u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(robot_state_.tau_J.data());
		}

		//call the mpc solver
		states_now_ << q_now_, v_now_;
		prob_->Solve_and_update_C_d_for_solver_errCoord(states_now_, )




        // Compute torques via QP and send them
//        for (size_t i = 0; i < NUM_JOINTS; i++) {
//            joint_handles_[i].setCommand(tau_command[i]);
//        }

    }

    // Some callback function required for us to send a custom message type to it
//    void QPController::desiredStateCallback(const QPController::DesiredState::ConstPtr& msg) {
//        // TODO - Would be good to impose safety limits on commanded velocity
//        // as well as keeping commanded position within joint limits
//        for(size_t i = 0; i < NUM_JOINTS; i++) {
//            desired_positions_[i] = msg->positions[i];
//            desired_velocities_[i] = msg->velocities[i];
//            feedforward_acceleration_[i] = msg->accelerations[i];
//        }
//    }


	//#######################################################################################
    //void QPController::init_plant() 
    //{
        //// Initialize the plant here
		//plant_ptr_ = std::make_unique<MultibodyPlant<double>>(h_env_);
		//Parser parser(plant_ptr_.get());
		//parser.AddModels(panda_file_);
		//const auto& arm_base_frame = plant_ptr_->GetFrameByName("panda_link0");

        //auto X_W_base = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * PI / 180.),
                                      //Vector3<double>(0., -0.2, 0.));
		//plant_ptr_->WeldFrames(plant_ptr_->world_frame(), arm_base_frame, X_W_base);

		//plant_ptr_->Finalize();
		//plantAD_ptr_ = System<double>::ToAutoDiffXd( *(this->plant_ptr_) );
		//contextAD_ptr_ = plantAD_ptr_->CreateDefaultContext();
		//context_ptr_ = plant_ptr_->CreateDefaultContext();
    //}

	//#######################################################################################
    //void QPController::init_prog() 
    //{
        //// Initialize the prog here
		//nDecVar_ = Nh_ * (nx_ + nu_);
		//dx_vars_ = prog_.NewContinuousVariables(Nh_, nx_, "joint ds");
		//du_vars_ = prog_.NewContinuousVariables(Nh_, nu_, "joint du");

		///* flatten the decision variables */
		//MatrixDecisionVariable<Eigen::Dynamic, Eigen::Dynamic> temp(du_vars_.rows(), 
																	//du_vars_.cols() + dx_vars_.cols());
		//temp << du_vars_, dx_vars_;

		////this create a view, not making a copy
		//auto decVar_flat(Eigen::Map<const VectorDecisionVariable<Eigen::Dynamic>>(temp.data(), temp.size()));

		//// add cost integral cost
		//for (int i = 0; i < Nh_ - 1; ++i)
		//{
			//auto dx_i = dx_vars_.row(i);
			//auto du_i = du_vars_.row(i);

			//// fogot that var goes out of scope created in for loop
			//auto bx = Eigen::VectorXd::Zero(nx_);
			//auto cost = prog_.AddQuadraticCost(Q_, bx, dx_i);
			//cost.evaluator()->set_description("Integral cost on s. ");
			//auto bu = Eigen::VectorXd::Zero(nu_);
			//cost = prog_.AddQuadraticCost(R_, bu, du_i);
			//cost.evaluator()->set_description("Integral cost on u. ");
		//}

		//// add terminal cost
		//auto dx_f = dx_vars_.row(Nh_ - 1);
		//auto b = Eigen::VectorXd::Zero(nx_);
		///* this will error: prog_.AddCost(0.5 * (dx_f.transpose() * P_ * dx_f)); */
		//auto cost = prog_.AddQuadraticCost(P_, b, dx_f); 

		//// set up the constraints
		//C_cols_ = Nh_ * (nx_ + nu_);

		//if (u_entries_.size() == 0)
		//{
			//C_rows_ = Nh_ * nx_;
		//} else {
			//C_rows_ = Nh_*nx_ + Nh_*nu_;
		//}

		//C_ = Eigen::MatrixXd::Zero(C_rows_, C_cols_);
		//C_.block(0, nu_, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);

		//lb_ = Eigen::VectorXd::Zero(C_rows_);
		//ub_ = Eigen::VectorXd::Zero(C_rows_);

		//cst_ = prog_.AddLinearConstraint(C_, lb_, ub_, decVar_flat);
    //}

	//#######################################################################################
	void QPController::joint_state_callback_sim(const sensor_msgs::JointState::ConstPtr& msg) 
	{
		//store current state
		// get the current joint position and velocity
		q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
		v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
		//u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->effort.data());
	}

	//#######################################################################################
	bool QPController::initial_pose_ok()
	{
		return true;
	}

} //namespace linearmpc_panda

PLUGINLIB_EXPORT_CLASS(linearmpc_panda::QPController, controller_interface::ControllerBase)

