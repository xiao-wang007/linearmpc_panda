// #include "mpc_executor_node.h"
#include <linearmpc_panda/mpc_executor_node.h>

namespace MPCControllers
{
    //########################################################################################
    MPCExecutorNode::MPCExecutorNode()
    {
        //fetch the first u_ref from the latched publisher 
        init_u_ref_sub_ = nh_.subscribe("/init_u_ref", 1, &MPCExecutorNode::fetch_init_u_ref, this);

        // Subscribe to the MPC solution topic
        mpc_sol_sub_ = nh_.subscribe("/mpc_sol", 1, &MPCExecutorNode::mpc_sol_callback, this);

        // sub to global mpc start time
        mpc_start_time_sub_ = nh_.subscribe("/mpc_t_start", 1, &MPCExecutorNode::get_mpc_start_time, this); 

        // Publisher for the upsampled control commands
        u_cmd_upsampled_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/upsampled_sol_traj", 1);

        // Timer for publishing upsampled commands
        upsample_timer_ = nh_.createTimer(ros::Duration(1.0 / executor_frequency_), &MPCExecutorNode::publish_upsampled_command, this);

        // init the matrix with ref traj of length Nh_*h_mpc_ just to cover before the first solution is ready
        u_sol_ = Eigen::MatrixXd::Zero(nu_, Nt_);

        Nt_ = Nh_ + 1; // Number of time points

        u_cmd_now_ = Eigen::VectorXd::Zero(nu_); // Initialize the upsampled command

        ROS_INFO("\n MPCExecutorNode initialized. $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ \n");
    }

    //########################################################################################
    void MPCExecutorNode::mpc_sol_callback(const linearmpc_panda::StampedFloat64MultiArray::ConstPtr& msg) 
    {
        std::lock_guard<std::mutex> lock(sol_spline_mutex_);

        // Extract the MPC solution from the message
        rows_ = msg->data.layout.dim[0].size;
        cols_ = msg->data.layout.dim[1].size;

        std::cout << "Received MPC solution with dimensions: " << rows_ << " x " << cols_ << std::endl;

        assert(rows_ == nu_ && cols_ == Nh_ && "MPC solution dimensions mismatch in executor node!");

        std::cout << "msg->data.data: ";
        for (const auto& value : msg->data.data) {
            std::cout << value << " ";
        }
        std::cout << std::endl;

        u_sol_ = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
            msg->data.data.data(), rows_, cols_);
        
        std::cout << "u_sol_: \n" << u_sol_ << std::endl;

        // Create time points for the spline, using stamped time
        auto t_now = ros::Time::now();
        current_time_ = (t_now - t_mpc_start_).toSec();
        Eigen::VectorXd ts = Eigen::VectorXd::LinSpaced(Nh_, current_time_, current_time_+Nh_*h_mpc_);

        // Create a PiecewisePolynomial spline for the control inputs
        u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(ts, u_sol_);
    }

    //########################################################################################
    void MPCExecutorNode::publish_upsampled_command(const ros::TimerEvent& event) 
    {        

        std::lock_guard<std::mutex> lock(sol_spline_mutex_);
        
        if (u_cmd_spline_.empty()) {

            ROS_WARN_THROTTLE(1.0, "No spline data available yet.");
            return;
        }


        // Get the current time relative to the spline
        auto t_now = ros::Time::now();

        // Evaluate the spline to get the upsampled control command
        // std::cout << "u_cmd_now_: \n" << u_cmd_now_.transpose() << std::endl;
        // std::cout << "u_cmd_spline_: \n" << u_cmd_spline_.value(0.).transpose() << std::endl;
        // std::cout << "t_now - t_mpc_start_: " << t_now - t_mpc_start_ << std::endl;
        // std::cout << "(t_now - t_mpc_start_).toSec(): " << (t_now - t_mpc_start_).toSec() << std::endl;

        u_cmd_now_ = u_cmd_spline_.value((t_now - t_mpc_start_).toSec());

        //ROS_INFO_STREAM("Publishing upsampled command u_cmd_now: " << u_cmd_now_.transpose());

        // Publish the upsampled command
        upsampled_msg_.data.resize(u_cmd_now_.size());
        Eigen::VectorXd::Map(upsampled_msg_.data.data(), u_cmd_now_.size()) = u_cmd_now_;

        u_cmd_upsampled_pub_.publish(upsampled_msg_);
    }

    //########################################################################################
    void MPCExecutorNode::run() 
    {
        ROS_INFO("[MPCExecutorNode] Waiting for /clock to start...");
        while (ros::Time::now().toSec() == 0.0 && ros::ok()) 
        {
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("[MPCExecutorNode] Clock started at time: %f", ros::Time::now().toSec());

        ros::spin();
    }

    //########################################################################################
    void MPCExecutorNode::get_mpc_start_time(const std_msgs::Time::ConstPtr& msg) 
    {
        // Lock the mutex to ensure thread safety
        std::lock_guard<std::mutex> lock(mpc_t_mutex_);
        t_mpc_start_ = msg->data;
        ROS_INFO("mpc_solver_node::get_mpc_start_time() runs once");
    }

    //########################################################################################
    void MPCExecutorNode::fetch_init_u_ref(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(sol_spline_mutex_);

        u_sol_.resize(nu_, Nt_);
        u_sol_ = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
            msg->data.data(), nu_, Nt_);

        std::cout << "<below> u_sol_.shape(): " << u_sol_.rows() << " x " << u_sol_.cols() << std::endl;
        
        auto t_now = ros::Time::now();
        current_time_ = t_now.toSec();
        
        std::cout << "t_now: " << current_time_ << std::endl; 

        Eigen::VectorXd ts = Eigen::VectorXd::LinSpaced(Nt_, current_time_, current_time_+Nh_*h_mpc_);
        std::cout << "ts.shape(): " << ts.rows() << " x " << ts.cols() << std::endl;

        // Create a PiecewisePolynomial spline for the control inputs
        u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(ts, u_sol_);

        std::cout << "End of fetch_init_u_ref. XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX " << std::endl;
    }

    //########################################################################################
	void MPCExecutorNode::waitForControllerToBeRunning(const std::string& controller_name) 
	{
		ROS_INFO_STREAM("Waiting for controller '" << controller_name << "' to be running...");

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
}

//########################################################################################
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mpc_executor_node");
    MPCControllers::MPCExecutorNode mpc_executor_node;
    mpc_executor_node.run();
    return 0;
}