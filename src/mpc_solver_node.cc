// #include "mpc_solver_node.h"
#include <linearmpc_panda/mpc_solver_node.h>

namespace MPCControllers
{
    //###############################################################################
    MPCSolverNode::MPCSolverNode() 
    {
		list_client_ = nh_.serviceClient<controller_manager_msgs::ListControllers>("/controller_manager/list_controllers");

        // load x_ref and u_ref
        data_proc_ = MyUtils::ProcessSolTraj(ref_traj_path_ , var_names_, dims_, times_);

        // init latched publisher 
        init_u_ref_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/init_u_ref", 1, true); //True for latched publisher
        std_msgs::Float64MultiArray init_u_ref_msg;


        init_u_ref_msg.data.resize(nu_ * Nt_);  
        // map the data into the message
        auto ts = Eigen::VectorXd::LinSpaced(Nt_, 0., Nh_ * h_mpc_);
        // std::cout << "data_proc_.u_ref_spline.vector_values(ts): \n" << data_proc_.u_ref_spline.vector_values(ts) << std::endl;
        Eigen::Map<Eigen::MatrixXd>(init_u_ref_msg.data.data(), nu_, Nt_) = data_proc_.u_ref_spline.vector_values(ts);
        //std::cout << "init_u_ref_msg.data: " << init_u_ref_msg.data << std::endl;

        init_u_ref_msg.layout.dim.resize(2);
        init_u_ref_msg.layout.dim[0].label = "rows";
        init_u_ref_msg.layout.dim[0].size = nu_;
        init_u_ref_msg.layout.dim[0].stride = nu_ * Nt_; // assuming row-major here
        init_u_ref_msg.layout.dim[1].label = "cols";
        init_u_ref_msg.layout.dim[1].size = Nt_;
        init_u_ref_msg.layout.dim[1].stride = Nt_;

        init_u_ref_pub_.publish(init_u_ref_msg);
        ROS_INFO("Published latched /init_u_ref inside MPCSolverNode(). \n");


        //solver_timer_ = nh.createTimer(ros::Duration(0.01), &MPCSolverNode::solve_and_update, this); // 100Hz
        mpc_sol_pub_ = nh_.advertise<linearmpc_panda::StampedFloat64MultiArray>("/mpc_sol", 1);

        mpc_start_time_ = nh_.subscribe("/mpc_t_start", 1, &MPCSolverNode::get_mpc_start_time, this); 

        /* Direct access to panda's current state, no need to use a sub, but here in gazebo, I need to do this through ros */
        //get panda state, topic belongs to franka_gazebo, which operates as 1kHz, then the callback is also called at 1kHz
        state_sub_ = nh_.subscribe("/joint_states", 1, &MPCSolverNode::joint_state_callback_sim, this);

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
        latest_mpc_sol_msg_.data.layout.dim[0].stride = nu_ * Nt_; // assuming row-major here
        latest_mpc_sol_msg_.data.layout.dim[1].label = "cols";
        latest_mpc_sol_msg_.data.layout.dim[1].size = Nt_;
        latest_mpc_sol_msg_.data.layout.dim[1].stride = Nt_;
        latest_mpc_sol_msg_.data.data.resize(nu_ * Nt_);  // Preallocate

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
        u_ref_cmd_ = Eigen::MatrixXd::Zero(nu_, Nt_);

        //print the initial position
        std::cout << "q0: \n" << data_proc_.x_ref_spline.value(0.).transpose() << std::endl;

        ROS_INFO("\n MPCSolverNode initialized. $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ \n");

    }

    //########################################################################################
    void MPCSolverNode::run() 
    {
        //
        //waitForControllerToBeRunning("franka_state_controller");

        t_start_node_ = std::chrono::high_resolution_clock::now();

        ROS_INFO("[MPCSolverNode] Waiting for /clock to start...");
        while (ros::Time::now().toSec() == 0.0 && ros::ok()) 
        {
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("[MPCSolverNode] Clock started at time: %f", ros::Time::now().toSec());

        //start solver in a separate thread
        std::thread solver_thread (
            [this]()
                {
                    while (ros::ok()) 
                    {
                        solve_publish_and_update();
                        // optional sleep to control the rate of the solver
                        //std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        //ros::Duration(0.01).sleep();
                    }
                }
        );

        ros::spin();

        // ros::spin() blocks this, then prints when ros is shutdown
        std::cout << "done spin() in MPCSolverNode::run()." << std::endl;

        solver_thread.join(); // Wait for the solver thread to finish
    }

    void MPCSolverNode::get_mpc_start_time(const std_msgs::Time::ConstPtr& msg) 
    {
        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(mpc_t_mutex_);
        t_mpc_start_ = msg->data;
        ROS_INFO("mpc_solver_node::get_mpc_start_time() runs once");
    }

    //########################################################################################
    void MPCSolverNode::solve_publish_and_update() 
    {
        // Lock the mutex to ensure thread safety
        Eigen::VectorXd local_q_now_, local_v_now_;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            local_q_now_ = q_now_;
            local_v_now_ = v_now_;
        } // lock release when go out of this scope

        //call the mpc solver
        state_now_ << local_q_now_, local_v_now_; // these are updated in the subcription callback
        std::cout << "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ \n" << std::endl;
        std::cout << "[MPCSolverNode] state_now_: \n" << state_now_.transpose() << std::endl;

        auto t_now_ = ros::Time::now();
        auto t_now_chro = std::chrono::high_resolution_clock::now();
        // current_time_ = (t_now_ - t_mpc_start_).toSec();
        current_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_now_chro - t_start_node_).count();
        std::cout << "t_mpc_start_: " << t_mpc_start_<< std::endl;
        // std::cout << "t_now_: " << t_now_ << std::endl;
        std::cout << "time-elipsed since node started: " << current_time_ << std::endl;
        //std::cout << "t_now_: " << t_now_ << std::endl;
        prob_->Solve_and_update_C_d_for_solver_errCoord(state_now_, current_time_);

        //get time here for stamping the message
        latest_mpc_sol_msg_.header.stamp = ros::Time::now();
        latest_mpc_sol_msg_.header.frame_id = "mpc_sol";

        //save solution to member variable u_ref_cmd_
        prob_->Get_solution(u_ref_cmd_); //Pass by argument

        // std::cout << "u_ref_cmd_ of shape: (" << u_ref_cmd_.rows() << ", " << u_ref_cmd_.cols() << ")" << std::endl;
        // std::cout << "u_ref_cmd_: \n" << u_ref_cmd_ << std::endl;

        //map solution to linearmpc_panda::StampedFloat64MultiArray 

        Eigen::Map<Eigen::MatrixXd>(latest_mpc_sol_msg_.data.data.data(), nu_, Nt_) = u_ref_cmd_;

        //publish the solution message
        mpc_sol_pub_.publish(latest_mpc_sol_msg_);

        ROS_INFO("mpc_solver_node::solve_and_update() runs once");
    }

    //########################################################################################
    void MPCSolverNode::joint_state_callback_sim(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        //store current state
        // get the current joint position and velocity
        // std::cout << "Checking inside joint_state_callback_sim() **********************************************" << std::endl;
        q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
        // std::cout << "q_now_: \n" << q_now_.transpose() << std::endl;
        v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
        // std::cout << "v_now_: \n" << v_now_.transpose() << std::endl;
        //u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->effort.data());
    }

    //########################################################################################
    void MPCSolverNode::joint_state_callback_HW(const sensor_msgs::JointState::ConstPtr& msg) 
    {
        //store current state
        // get the current joint position and velocity
        std::mutex state_mutex_;
        q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
        v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
        //u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->effort.data());
    }

    //########################################################################################
	void MPCSolverNode::waitForControllerToBeRunning(const std::string& controller_name) 
	{
		ROS_INFO_STREAM("Waiting inside [MPCSolverNode] for controller '" << controller_name << "' to be running...xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");

		while (ros::ok()) 
		{
			controller_manager_msgs::ListControllers srv;
			if (list_client_.call(srv)) 
			{
				for (const auto& controller : srv.response.controller) 
				{
					if (controller.name == controller_name && controller.state == "running") 
					{
						ROS_INFO_STREAM("Controller '" << controller_name << "' is running.");
						return;
					}
				}
			} else {
				ROS_WARN_THROTTLE(5.0, "Could not call /controller_manager/list_controllers yet...");
			}

			ros::Duration(0.5).sleep();
		}
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