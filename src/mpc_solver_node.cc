#include "mpc_solver_node.h"

//###############################################################################
MPCSolverNode::MPCSolverNode() 
{
    // Initialize the ROS node
    ros::NodeHandle nh;
    solver_timer_ = nh.createTimer(ros::Duration(0.01), &MPCSolverNode::run, this);
    mpc_sol_pub_ = nh.advertise<std_msgs::Float64MultiArray>("mpc_solution", 1);

	/* Direct access to panda's current state, no need to use a sub, but here in gazebo, I need to do this through ros */
	//get panda state, topic belongs to franka_gazebo, which operates as 1kHz, then the callback is also called at 1kHz
	state_sub_ = node_handle.subscribe("joint_states", 1, &MPCSolverNode::joint_state_callback_sim, this);

    /*TODO: create the publisher in QPController interface to publish panda hardware current state*/
	//state_sub_ = node_handle.subscribe("joint_states_pandaHW", 1, &MPCSolverNode::joint_state_callback_HW, this);


	//define the meta data of a std_msgs::Float64MultiArray for mpc_solution
	mpc_sol_msg_.layout.dim.resize(2);
	mpc_sol_msg_.layout.dim[0].label = "rows";
	mpc_sol_msg_.layout.dim[0].size = nu_;
	mpc_sol_msg_.layout.dim[0].stride = nu_ * Nh_; // assuming row-major here
	mpc_sol_msg_.layout.dim[1].label = "cols";
	mpc_sol_msg_.layout.dim[1].size = Nh_;
	mpc_sol_msg_.layout.dim[1].stride = Nh_;
	mpc_sol_msg_.data.resize(nu_ * Nh_);  // Preallocate

	// compute mpc related parameters, this has to go first as init_prog() uses them
	Nh_ = Nt_ - 1;
	execution_length_ = h_mpc_ * n_exe_steps_;
	mpc_horizon_ = h_mpc_ * Nh_;

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

}

//########################################################################################
MPCSolverNode::run() 
{
    ros::spin();
}

//########################################################################################
MPCSolverNode::solve_and_update(const ros::TimerEvent& event) 
{
    // Lock the mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(mpc_mutex_);

    //get current reference trajectory 
    t_now_ = ros::Time::now().toSec();
    ts_ = Eigen::VectorXd::LinSpaced(Nt_, t_now_, t_now_+mpc_horizon_);
    xref_now_ = data_proc_.x_ref_spline.vector_values(ts_);
    uref_now_ = data_proc_.u_ref_spline.vector_values(ts_);

    //call the mpc solver
    state_now_ << q_now_, v_now_; // these are updated in the subcription callback
    t_now_ = ros::Time::now().toSec();
    prob_->Solve_and_update_C_d_for_solver_errCoord(state_now_, t_now_, u_ref_cmd_);

    //map solution to std_msgs::Float64MultiArray 
    mpc_sol_msg_.data.resize(u_ref_cmd_.size());
    Eigen::Map<Eigen::MatrixXd>(mpc_sol_msg_.data.data(), nu_, Nh_) = u_ref_cmd_;

    //publish the solution message
    mpc_sol_pub_.publish(mpc_sol_msg_);
}

//########################################################################################
void MPCSolverNode::joint_state_callback_sim(const sensor_msgs::JointState::ConstPtr& msg) 
{
    //store current state
    // get the current joint position and velocity
    q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
    v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
    //u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->effort.data());
}

//########################################################################################
void MPCSolverNode::joint_state_callback_sim(const sensor_msgs::JointState::ConstPtr& msg) 
{
    //store current state
    // get the current joint position and velocity
    q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
    v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
    //u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->effort.data());
}

