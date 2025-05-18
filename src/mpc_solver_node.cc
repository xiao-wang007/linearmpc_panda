// #include "mpc_solver_node.h"
#include <linearmpc_panda/mpc_solver_node.h>

namespace MPCControllers
{
    //###############################################################################
    MPCSolverNode::MPCSolverNode() 
    {
        //solver_timer_ = nh.createTimer(ros::Duration(0.01), &MPCSolverNode::solve_and_update, this); // 100Hz
        mpc_sol_pub_ = nh_.advertise<linearmpc_panda::StampedFloat64MultiArray>("mpc_sol", 1);

        mpc_start_time_ = nh_.subscribe("mpc_t_start", 1, &MPCSolverNode::get_mpc_start_time, this); //True for latched publisher

        /* Direct access to panda's current state, no need to use a sub, but here in gazebo, I need to do this through ros */
        //get panda state, topic belongs to franka_gazebo, which operates as 1kHz, then the callback is also called at 1kHz
        state_sub_ = nh_.subscribe("joint_states", 1, &MPCSolverNode::joint_state_callback_sim, this);

        /*TODO: create the publisher in QPController interface to publish panda hardware current state*/
        //state_sub_ = nh.subscribe("joint_states_pandaHW", 1, &MPCSolverNode::joint_state_callback_HW, this);

        // compute mpc related parameters, this has to go first as init_prog() uses them
        Nh_ = Nt_ - 1;
        execution_length_ = h_mpc_ * n_exe_steps_;
        mpc_horizon_ = h_mpc_ * Nh_;

        //define the meta data of a std_msgs::Float64MultiArray for mpc_solution
        latest_mpc_sol_msg_.data.layout.dim.resize(2);
        latest_mpc_sol_msg_.data.layout.dim[0].label = "rows";
        latest_mpc_sol_msg_.data.layout.dim[0].size = nu_;
        latest_mpc_sol_msg_.data.layout.dim[0].stride = nu_ * Nh_; // assuming row-major here
        latest_mpc_sol_msg_.data.layout.dim[1].label = "cols";
        latest_mpc_sol_msg_.data.layout.dim[1].size = Nh_;
        latest_mpc_sol_msg_.data.layout.dim[1].stride = Nh_;
        latest_mpc_sol_msg_.data.data.resize(nu_ * Nh_);  // Preallocate


        // load x_ref and u_ref
        data_proc_ = MyUtils::ProcessSolTraj(ref_traj_path_ , var_names_, dims_, times_);

        //init MyControllers::LinearMPCProb() here
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
        prob_ = std::make_unique<MPCControllers::LinearMPCProb>(panda_file_, integrator_, nx_, nu_, execution_length_, 
                                                            h_mpc_, h_env_, Nt_, X_W_base_, Q, R, P, 
                                                            data_proc_.x_ref_spline, 
                                                            data_proc_.u_ref_spline); 
        //init solver output
        state_now_ = Eigen::VectorXd::Zero(nx_);
        u_ref_cmd_ = Eigen::MatrixXd::Zero(nu_, Nh_);

        //print the initial position
        std::cout << "q0: \n" << data_proc_.x_ref_spline.value(0.).transpose() << std::endl;

    }

    //########################################################################################
    void MPCSolverNode::run() 
    {
        while (ros::ok()) 
        {
            // Call the solve_and_update function at a fixed rate
            solve_publish_and_update();

            // no sleep, event driven
            //ros::Duration(0.01).sleep(); // 100Hz
        ros::spinOnce();
        }
    }

    void MPCSolverNode::get_mpc_start_time(const std_msgs::Time::ConstPtr& msg) 
    {
        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(mpc_mutex_);
        t_mpc_start_ = msg->data;
        ROS_INFO("mpc_solver_node::get_mpc_start_time() runs once");
    }

    //########################################################################################
    void MPCSolverNode::solve_publish_and_update() 
    {
        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(mpc_mutex_);

        ////get current reference trajectory for debugging 
        //t_now_ = ros::Time::now();
        //auto t_start = (t_now_-t_init_node_).toSec();
        //auto ts_ = Eigen::VectorXd::LinSpaced(Nt_, t_start, t_start+mpc_horizon_);
        //xref_now_ = data_proc_.x_ref_spline.vector_values(ts_);
        //uref_now_ = data_proc_.u_ref_spline.vector_values(ts_);

        //std::cout << "uref_now_: \n" << uref_now_ << std::endl;

        //call the mpc solver
        state_now_ << q_now_, v_now_; // these are updated in the subcription callback
        t_now_ = ros::Time::now();
        current_time_ = (t_mpc_start_ - t_now_).toSec();
        prob_->Solve_and_update_C_d_for_solver_errCoord(state_now_, t_now_);

        //get time here for stamping the message
        latest_mpc_sol_msg_.header.stamp = ros::Time::now();
        latest_mpc_sol_msg_.header.frame_id = "mpc_sol";

        //save solution to member variable u_ref_cmd_
        prob_->Get_solution(u_ref_cmd_); //Pass by argument

        std::cout << "u_ref_cmd_ of shape: (" << u_ref_cmd_.rows() << ", " << u_ref_cmd_.cols() << ")" << std::endl;
        std::cout << "u_ref_cmd_: \n" << u_ref_cmd_ << std::endl;

        //map solution to linearmpc_panda::StampedFloat64MultiArray 
        Eigen::Map<Eigen::MatrixXd>(latest_mpc_sol_msg_.data.data.data(), nu_, Nh_) = u_ref_cmd_;

        //publish the solution message
        mpc_sol_pub_.publish(latest_mpc_sol_msg_);

        ROS_INFO("mpc_solver_node::solve_and_update() runs once");
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
    void MPCSolverNode::joint_state_callback_HW(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        //store current state
        // get the current joint position and velocity
        q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
        v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
        //u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->effort.data());
    }

} // namespace MPCControllers


//########################################################################################
int main(int argc, char** argv) 
{
    // Initialize the ROS node
    ros::init(argc, argv, "mpc_solver_node");
    
    // Create an instance of the MPCSolverNode class
    MPCControllers::MPCSolverNode mpc_solver_node;
    
    // Run the node
    mpc_solver_node.run();
    
    return 0;
}