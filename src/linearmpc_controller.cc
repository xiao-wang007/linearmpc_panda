#include <linearmpc_panda/linearmpc_controller.h>


namespace MyControllers
{
    //###############################################################################
    LinearMPCControllerNode::LinearMPCControllerNode()
    {
        upsampled_u_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/upsampled_u_cmd", 1);

        state_sub_ = nh_.subscribe("/franka_state_controller/joint_states", 1, &LinearMPCControllerNode::joint_state_callback, this);
        upsample_timer_ = nh_.createTimer(ros::Duration(1.0 / executor_frequency_), &LinearMPCControllerNode::publish_upsampled_command, this);

        Nh_ = Nt_ - 1;
        execution_length_ = h_mpc_ * n_exe_steps_;
        mpc_horizon_ = h_mpc_ * Nh_;

        X_W_base_ = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * PI / 180.),
                                        Vector3<double>(0., -0.2, 0.));

        // load reference trajectory data
        data_proc_ = MyUtils::ProcessSolTraj(ref_traj_path_ , var_names_, dims_, times_);

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
        latest_mpc_sol_ = Eigen::MatrixXd::Zero(nu_, Nt_);

        //init u_sol_spline_ by the first bit before mpc solution is ready
        auto ts = Eigen::VectorXd::LinSpaced(Nt_, 0., Nh_ * h_mpc_);
        u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(ts, data_proc_.u_ref_spline.vector_values(ts));
        
        
        ROS_INFO("MPCControllerNode initialized successfully!\n");
    }

    //###############################################################################
    void LinearMPCControllerNode::joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
        v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());

        if (!received_first_state_)
        {
            received_first_state_ = true; // Set the flag to true after receiving the first state
            ROS_INFO("Received first joint state from simulation.");
            ROS_INFO_STREAM("Current joint positions: " << q_now_.transpose());
            ROS_INFO_STREAM("Current joint velocities: " << v_now_.transpose());
            auto wall_time = ros::WallTime::now();
            std::cout << "[linearmpc_controller] wall time now: " << wall_time << std::endl;
        }
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


        if (!mpc_started_) 
        {
            mpc_t_start_ = ros::Time::now();
            mpc_started_ = true;
            ROS_INFO_STREAM("[linearmpc_controller] MPC started, t_start initialized: t_start = " << mpc_t_start_);
        }

        Eigen::VectorXd local_q_now, local_v_now;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            local_q_now = q_now_;
            local_v_now = v_now_;
        } // lock release when go out of this scope

        state_now_ << local_q_now, local_v_now; // these are updated in the subcription callback
        std::cout << "q_now: " << local_q_now.transpose() << std::endl;

        auto t_now = ros::Time::now();
        // auto t_now_chro = std::chrono::high_resolution_clock::now();
        // current_time_ = std::chrono::duration_cast<std::chrono::duration<double>>(t_now_chro - t_start_node_).count();
        auto current_time = (t_now - mpc_t_start_).toSec();
        std::cout << "[linearmpc_controller] time lapsed since node started: " << current_time << std::endl;

        prob_->Solve_and_update_C_d_for_solver_errCoord(state_now_, current_time); // takes 0.22~2.45s

        prob_->Get_solution(latest_mpc_sol_); //Pass by argument
        std::cout << "[linearmpc_controller] latest_mpc_sol_: \n" << latest_mpc_sol_ << std::endl;
        u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
                Eigen::VectorXd::LinSpaced(Nt_, current_time, current_time + mpc_horizon_), latest_mpc_sol_);

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
        Eigen::VectorXd upsampled_u_cmd = u_cmd_spline_copy.value((t_now - mpc_t_start_).toSec());
        // std::cout << "[linearmpc_controller] upsampled_u_cmd: " << upsampled_u_cmd.transpose() << std::endl;
        
        std_msgs::Float64MultiArray msg;
        msg.data = std::vector<double>(upsampled_u_cmd.data(), upsampled_u_cmd.data() + upsampled_u_cmd.size());
        upsampled_u_cmd_pub_.publish(msg);
    }

    //###############################################################################
    void LinearMPCControllerNode::run()
    {
        //ROS_INFO("Waiting for the simulation to be ready...");

        //while (ros::ok() && !simulation_ready_signal_)
        //{
            //nh_.getParam("/simulation_ready", simulation_ready_signal_);
            //if (!simulation_ready_signal_)
            //{
                //std::cout << "Simulation not ready yet, waiting..." << std::endl;
                //ros::Duration(0.1).sleep(); // Sleep for a short duration to avoid busy-waiting
            //}
        //}

        //ROS_INFO("Simulation is ready!");

        //start solver in a separate thread
        std::thread solver_thread (
            [this]()
                {
                    while (ros::ok()) 
                    {
                        try 
                        { 
                            auto start = std::chrono::high_resolution_clock::now();

                            solve_and_update(); 

                            auto end = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> elapsed = end - start;
                            std::cout << "[linearmpc_controller] solve_and_update took: " << elapsed.count() << " seconds." << std::endl;
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

        // ros::spin() blocks this, then prints when ros is shutdown
        std::cout << "done spin() in MPCSolverNode::run()." << std::endl;

        solver_thread.join(); // Wait for the solver thread to finish
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "linearmpc_controller_node");
    MyControllers::LinearMPCControllerNode mpc_controller_node;
    mpc_controller_node.run();
    return 0;
}