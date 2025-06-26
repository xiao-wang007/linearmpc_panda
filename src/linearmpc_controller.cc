#include <linearmpc_panda/linearmpc_controller.h>


namespace MyControllers
{
    //###############################################################################
    LinearMPCControllerNode::LinearMPCControllerNode(ros::NodeHandle& node_handle) : nh_(node_handle)
    {
        //subs, pubs and services
        upsampled_u_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/upsampled_u_cmd", 1);
        state_sub_ = nh_.subscribe("/franka_state_controller/joint_states", 1, &LinearMPCControllerNode::joint_state_callback, this);
        upsample_timer_ = nh_.createTimer(ros::Duration(1.0 / executor_frequency_), &LinearMPCControllerNode::publish_upsampled_command, this);
        //q_init_reached_sub_ = nh_.subscribe("/q_init_reached", 1, &LinearMPCControllerNode::q_init_reached_callback, this);
        move_to_pose_client_ = nh_.serviceClient<std_srvs::Trigger>("move_to_pose");

        //
        if (!nh_.getParam("exclude_gravity", exclude_gravity_))
        {
            ROS_INFO("LinearMPCControllerNode: exclude_gravity_ not set, defaulting to false.");
            exclude_gravity_ = false; // default value
        }

        //
        if (!nh_.getParam("decVar_bounds_on", decVar_bounds_on_))
        {
            ROS_INFO("LinearMPCControllerNode: decVar_bounds_on_ not set, defaulting to true.");
            decVar_bounds_on_ = true; // default value
        }

        //
        if (!nh_.getParam("udot_bounds_on", udot_bounds_on_))
        {
            ROS_INFO("LinearMPCControllerNode: udot_bounds_on_ not set, defaulting to true.");
            udot_bounds_on_ = true; // default value
        }

        //
        if (!nh_.getParam("Nh", Nh_))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter Nh_.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: Nh_ parameter not set."); 
        }
        Nt_ = Nh_ + 1;

        //
        if (!nh_.getParam("h_mpc", h_mpc_))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter h_mpc_.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: h_mpc_ parameter not set.");
        }
        mpc_horizon_ = h_mpc_ * Nh_;

        // set up the upper and lower bounds for u and x
        std::vector<double> u_up_vec;
        if (!nh_.getParam("u_upper_bounds", u_up_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter u_up_.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: u_up_ not set.");
        }
        u_up_ = Eigen::Map<Eigen::VectorXd>(u_up_vec.data(), u_up_vec.size());
        //u_up_.resize(nu_);
        //u_up_ << 87., 87., 87., 87., 12., 12., 12.;

        //
        std::vector<double> u_low_vec;
        if (!nh_.getParam("u_lower_bounds", u_low_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter u_low_.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: u_low_ not set.");
        }
        u_low_ = Eigen::Map<Eigen::VectorXd>(u_low_vec.data(), u_low_vec.size());
        //u_low_.resize(nu_);
        //u_low_ << -87., -87., -87., -87., -12., -12., -12.;

        //
        std::vector<double> x_up_vec;
        if (!nh_.getParam("x_up_bounds", x_up_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter x_up_.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: x_up_ not set.");
        }
        x_up_ = Eigen::Map<Eigen::VectorXd>(x_up_vec.data(), x_up_vec.size());
        //x_up_.resize(nx_);
        //x_up_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 2.1750, 2.1750, 2.1750, 2.1750, 2.61, 2.61, 2.61;


        //
        std::vector<double> x_low_vec;
        if (!nh_.getParam("x_low_bounds", x_low_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter x_low_.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: x_low_ not set.");
        }
        x_low_ = Eigen::Map<Eigen::VectorXd>(x_low_vec.data(), x_low_vec.size());
        //x_low_.resize(nx_);
        //x_low_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, -2.1750, -2.1750, -2.1750, -2.1750, -2.61, -2.61, -2.61;

        //
        std::vector<double> x_entries_vec;
        if (!nh_.getParam("x_entries", x_entries_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter x_entries.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: x_entries not set.");
        }
        x_entries_ = Eigen::Map<Eigen::VectorXd>(x_entries_vec.data(), x_entries_vec.size());
        //x_entries_ = Eigen::VectorXd::Ones(nx_);

        //
        std::vector<double> u_entries_vec;
        if (!nh_.getParam("u_entries", u_entries_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter u_entries.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: u_entries not set.");
        }
        u_entries_ = Eigen::Map<Eigen::VectorXd>(u_entries_vec.data(), u_entries_vec.size());
        //u_entries_ = Eigen::VectorXd::Ones(nu_);

        //
        std::vector<double> roll_pitch_yaw_vec;
        if (!nh_.getParam("base_roll_pitch_yaw", roll_pitch_yaw_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter roll_pitch_yaw.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: roll_pitch_yaw not set.");
        }
        std::vector<double> position_vec;
        if (!nh_.getParam("base_position", position_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter base position.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: base position not set.");
        }
        X_W_base_ = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(roll_pitch_yaw_vec[0] * PI / 180.,
                                                                                roll_pitch_yaw_vec[1] * PI / 180.,
                                                                                roll_pitch_yaw_vec[2] * PI / 180.)),
                                           Vector3<double>(position_vec[0], position_vec[1], position_vec[2])); 
        //X_W_base_ = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * PI / 180.),
                                        //Vector3<double>(0., -0.2, 0.));

        //make Q
        std::vector<double> Q_diag_vec;
        if (!nh_.getParam("Q_matrix_coeffs", Q_diag_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter Q matrix coeffs.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: Q matrix coeffs not set.");
        }
        Q_diag_vec_ = Eigen::Map<Eigen::VectorXd>(Q_diag_vec.data(), Q_diag_vec.size());

        ////Eigen::VectorXd q_coef = Eigen::VectorXd::Constant(nu_, 200.0) * 12.;
        ////Eigen::VectorXd v_coef = Eigen::VectorXd::Constant(nu_, 1.) * 0.1;
        //Eigen::VectorXd q_coef = Eigen::VectorXd::Constant(nu_, 10.0) * 12.;
        //Eigen::VectorXd v_coef = Eigen::VectorXd::Constant(nu_, 1.) * 0.1;
        //Eigen::VectorXd Q_diags(q_coef.rows() + v_coef.rows());
        //Q_diags.head(q_coef.rows()) = q_coef;
        //Q_diags.tail(v_coef.rows()) = v_coef;
        //Eigen::DiagonalMatrix<double, NUM_JOINTS*2> Q_sparse = Q_diags.asDiagonal();
        //Q_ = Q_sparse.toDenseMatrix();

        //make R
        std::vector<double> R_diag_vec;
        if (!nh_.getParam("R_matrix_coeffs", R_diag_vec))
        {
            ROS_ERROR_STREAM("LinearMPCControllerNode: Failed to get parameter R matrix coeffs.");
            throw std::runtime_error("Failed to initialize LinearMPCControllerNode: R matrix coeffs not set.");
        }
        R_diag_vec_ = Eigen::Map<Eigen::VectorXd>(R_diag_vec.data(), R_diag_vec.size());
        ////Eigen::VectorXd u_coef = Eigen::VectorXd::Constant(nu_, 0.001);
        //Eigen::VectorXd u_coef = Eigen::VectorXd::Constant(nu_, 10);
        //R_ = u_coef.asDiagonal();

        // load reference trajectory data
        if (exclude_gravity_)
        {
            auto plant_ptr = std::make_unique<MultibodyPlant<double>>(h_env_);
            Parser parser(plant_ptr.get());
            parser.AddModels(panda_file_);
            const auto& arm_base_frame = plant_ptr->GetFrameByName("panda_link0");
            plant_ptr->WeldFrames(plant_ptr->world_frame(), arm_base_frame, X_W_base_);
            plant_ptr->Finalize();
            auto context_ptr = plant_ptr->CreateDefaultContext();

            //loop to subtract gravity from the reference trajectory
            data_proc_ = MyUtils::ProcessSolTraj(ref_traj_path_ , var_names_, dims_, times_);
            //std::cout << "data_proc_.trajs.at('q_panda').row(0): " << data_proc_.trajs.at("q_panda").row(1) << std::endl;
            //std::cout << "data_proc_.trajs.at('q_panda') of shape: " << data_proc_.trajs.at("q_panda").rows() << "x" 
                      //<< data_proc_.trajs.at("q_panda").cols() << std::endl;
            int N = data_proc_.trajs.at("us").rows();
            for(int i = 0; i < N; i++)
            {
                Eigen::VectorXd qi = data_proc_.trajs.at("q_panda").row(i);
                //compute G
                plant_ptr->SetPositions(context_ptr.get(), qi);
                /* was this in the motion planning: 
                        u = c - g - M@(v_next - v) 
                   now:
                        u = u + g */
                auto temp = data_proc_.trajs.at("us").row(i).transpose() + plant_ptr->CalcGravityGeneralizedForces(*context_ptr);
                data_proc_.trajs.at("us").row(i) = temp;
            }
        }
        else {
            data_proc_ = MyUtils::ProcessSolTraj(ref_traj_path_ , var_names_, dims_, times_);
        }

        /* If data_proc_.trajs.at("q_panda") is an Eigen::MatrixXd, then .row(0) returns an Eigen 
        row vector of type Eigen::Matrix<double, 1, Eigen::Dynamic>. If you assign this directly 
        to an Eigen::VectorXd, it can cause all elements to be set to the first value (due to 
        Eigen's implicit conversion rules).*/
        //get q_init_desired!!! 
        q_init_desired_ = data_proc_.trajs.at("q_panda").row(0).transpose();

        std::cout << "inside init() q_init_desired_: " << q_init_desired_.transpose() << std::endl;

        //Publish q_init_desired to be used in move_to_pose service
        q_init_desired_pub_ = nh_.advertise<sensor_msgs::JointState>("/q_init_desired", 1, true);
        sensor_msgs::JointState q_init_desired_msg;
        q_init_desired_msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", 
                                   "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
        q_init_desired_msg.position = std::vector<double>(q_init_desired_.data(), q_init_desired_.data() + q_init_desired_.size());
        q_init_desired_pub_.publish(q_init_desired_msg);
        ROS_INFO_STREAM("Published initial desired joint state: " << q_init_desired_);

        /* have to do this to avoid DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN() assertion as 
            I have a MultibodyPlant() inside LinearMPCProb()*/
        prob_ = std::make_unique<MPCControllers::LinearMPCProb>(panda_file_, 
                                                                integrator_, 
                                                                exclude_gravity_,
                                                                decVar_bounds_on_,
                                                                udot_bounds_on_,
                                                                N_,
                                                                nx_, 
                                                                nu_, 
                                                                h_mpc_, 
                                                                h_env_,
                                                                Nt_, 
                                                                X_W_base_, 
                                                                Q_diag_vec_, 
                                                                R_diag_vec_, 
                                                                data_proc_, 
                                                                u_up_, 
                                                                u_low_, 
                                                                x_up_, 
                                                                x_low_, 
                                                                u_entries_, 
                                                                x_entries_); 


        //init solver output
        latest_mpc_sol_ = Eigen::MatrixXd::Zero(nu_, Nh_);

        //init u_sol_spline_ by the first bit before mpc solution is ready
        auto ts = Eigen::VectorXd::LinSpaced(Nh_, 0., Nh_ * h_mpc_);
        u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(ts, data_proc_.u_ref_spline.vector_values(ts));
        
        ROS_INFO("MPCControllerNode initialized successfully!\n");
    }

    //###############################################################################
    void LinearMPCControllerNode::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
        v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
        
        received_first_state_ = true; // Set the flag to true after receiving the first state
    }

    //###############################################################################
    void LinearMPCControllerNode::solve_and_update()
    {
        if (!received_first_state_)
        {
            ROS_WARN_THROTTLE(2.0, "Waiting for first joint state before solving...");
            ros::Duration(0.01).sleep();
            return;
        }

        Eigen::VectorXd local_q_now, local_v_now;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            local_q_now = q_now_;
            local_v_now = v_now_;
        } // lock release when go out of this scope

        // smooth joint velocity measurements
        double alpha = 0.95;
        for (size_t i = 0; i < local_v_now.size(); ++i)
        {
            v_now_filtered_(i) = alpha * local_v_now(i) + (1 - alpha) * v_now_filtered_(i);
        }

        state_now_ << local_q_now, v_now_filtered_; // these are updated in the subcription callback
        std::cout << "q_now: " << local_q_now.transpose() << std::endl;

		DRAKE_DEMAND(!state_now_.hasNaN() && "state_now_ has NaN values!");

        auto t_now = ros::Time::now();
        // auto t_now_chro = std::chrono::high_resolution_clock::now();
        // current_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_now_chro - t_start_node_).count();
        std::cout << '\n' << std::endl;
        auto current_time = (t_now - mpc_t_start_).toSec();
        std::cout << "[linearmpc_controller] time lapsed since node started: " << current_time << std::endl;

        prob_->Solve_and_update_C_d_for_solver_errCoord(state_now_, current_time); // takes 0.22~0.245s

        prob_->Get_solution(latest_mpc_sol_); //Pass by argument

        //std::cout << "[linearmpc_controller] latest_mpc_sol_: \n" << latest_mpc_sol_ << std::endl;

        u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
                Eigen::VectorXd::LinSpaced(Nh_, current_time, current_time + mpc_horizon_), latest_mpc_sol_);
    }

    //###############################################################################
    void LinearMPCControllerNode::publish_upsampled_command(const ros::TimerEvent& event)
    {
        if (!received_first_state_) 
        {
            ROS_WARN_THROTTLE(2.0, "Waiting for first joint state before publishing upsampled command...");
            return;
        }
        drake::trajectories::PiecewisePolynomial<double> u_cmd_spline_copy;
        {
            // Lock the mutex to ensure thread safety
            std::lock_guard<std::mutex> lock(u_cmd_spline_mutex_);
            u_cmd_spline_copy = u_cmd_spline_;
        }

        auto t_now = ros::Time::now();
        //Eigen::VectorXd upsampled_u_cmd = u_cmd_spline_copy.value((t_now - mpc_t_start_).toSec());

        //Sending torques to just last joint
        Eigen::VectorXd upsampled_u_cmd = Eigen::VectorXd::Zero(nu_);
        upsampled_u_cmd(nu_ - 1) = u_cmd_spline_copy.value((t_now - mpc_t_start_).toSec())(nu_ - 1);

        /*
        //should not do this! this basically ignores Coriolis
        Eigen::VectorXd upsampled_u_cmd = Eigen::VectorXd::Zero(7);
        int active_joint_index = 0;
        upsampled_u_cmd(0) = u_cmd_spline_copy.value((t_now - mpc_t_start_).toSec())(active_joint_index); 
        */

        std_msgs::Float64MultiArray msg;
        msg.data = std::vector<double>(upsampled_u_cmd.data(), upsampled_u_cmd.data() + upsampled_u_cmd.size());
        upsampled_u_cmd_pub_.publish(msg);
    }

    //###############################################################################
    void LinearMPCControllerNode::run()
    {
        ros::Rate rate(10);

        while (ros::ok() && !received_first_state_)
        {
            ROS_WARN_THROTTLE(2.0, "Waiting for first joint state before checking initial pose...");
            ros::spinOnce();
            rate.sleep();
        }
        
        // wait for /q_init_reached to be true
        while (ros::ok() && !q_init_reached_)
        {
            if (!service_called_)
            {   
                std::cout << "checking if move_to_pose service is available..." << std::endl;
                std_srvs::Trigger srv;
                if (move_to_pose_client_.call(srv))
                {
                  if (srv.response.success)
                  {  
                    ROS_INFO_STREAM("Called move_to_pose service: " << srv.response.message);
                    service_called_ = true; 
                  }
                  else
                  {
                    ROS_WARN("Failed to call move_to_pose service, retrying...");
                  }
                } 

                else 
                {
                  ROS_WARN("Failed to call move_to_pose service, retrying...");
                }
            }

            q_init_reached_ = initial_pose_reached(); // check if the robot is at the desired initial pose
            if (q_init_reached_)
            {
                ROS_INFO("Robot is at the desired initial pose, starting MPC...*********************************");
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }

        mpc_t_start_ = ros::Time::now();

        //start solver in a separate thread
        std::thread solver_thread (
            [this]()
                {
                    while (ros::ok()) 
                    {
                        try 
                        { 
                            //auto start = std::chrono::high_resolution_clock::now();

                            solve_and_update(); 

                            //auto end = std::chrono::high_resolution_clock::now();
                            //std::chrono::duration<double> elapsed = end - start;
                            //std::cout << "[linearmpc_controller] solve_and_update took: " << elapsed.count() << " seconds." << std::endl;
                            //std::cout << "Current time: " << (ros::Time::now() - mpc_t_start_).toSec() << " seconds." << std::endl;
                        } 
                        catch (const std::bad_alloc& e) 
                        {
                            ROS_FATAL_STREAM("Memory allocation failed: " << e.what());
                        } 
                        catch (const std::exception& e) 
                        {
                         ROS_FATAL_STREAM("Exception in solve_and_update: " << e.what());
                        }
                    }
                }
        );

        //set a flag here to start spin when sim node starts to publish joint states
        ros::spin(); // spin here as solve_and_update() is called in a separate thread

        // 4. Clean shutdown
        if (solver_thread.joinable()) 
        {
            solver_thread.join();
        }

        std::cout << "LinearMPCControllerNode::run() finished cleanly." << std::endl;
    }

    //###############################################################################
    bool LinearMPCControllerNode::initial_pose_reached() 
	{
		// Check if the current robot state is close to the desired initial pose
		const double threshold = 0.01; // Adjust this threshold as needed

        Eigen::VectorXd local_q_now, local_v_now;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            local_q_now = q_now_;
            local_v_now = v_now_;
        } // lock release when go out of this scope
        std::cout << "local_q_now: " << local_q_now.transpose() << std::endl;


        //std::cout << "q_init_desired_: " << q_init_desired_.transpose() << std::endl;

		if ((local_q_now - q_init_desired_).norm() > threshold) 
		{
			ROS_WARN("Current robot state is not close to the desired initial pose.");
			return false;
		}
		else if ((local_q_now - q_init_desired_).norm() < threshold)
		{
			ROS_INFO("Current robot state is close to the desired initial pose.");
			return true;
		}

		ROS_INFO("Current robot state is exactly at the threshold.");
		return true;  // or false, depending on your logic
	}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "linearmpc_controller_node");
    ros::NodeHandle nh;

    try 
    {
        MyControllers::LinearMPCControllerNode mpc_controller_node(nh);
        mpc_controller_node.run();

    }
    catch (const std::exception& e)
    {
        ROS_FATAL_STREAM("Exception during iniitialization: " << e.what());
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}