// #include "panda_QP_controller.h"
#include <linearmpc_panda/panda_QP_controller.h>
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/hardware_interface.h>

namespace linearmpc_panda {

	//#######################################################################################
    bool LinearMPCController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) 
	{

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

		joint_handles_.clear();
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint1"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint2"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint3"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint4"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint5"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint6"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint7"));

		/* Direct access to panda's current state, no need to use a sub, but here in gazebo, I need to do this through ros */
		//get panda state, topic belongs to franka_gazebo, which operates as 1kHz, then the callback is also called at 1kHz
		//state_sub_ = node_handle.subscribe<sensor_msgs::JointState>("joint_states", 1, &LinearMPCController::joint_state_callback_sim, this);

		// Create a publisher for the Panda hardware state
		//state_pub_ = node_handle.advertise<sensor_msgs::JointState>("joint_state_pandaHW", 1);

        //// Convert current robot position and velocity into Eigen data storage
        //Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> q_now_(robot_state_.q.data());
        //Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> v_now_(robot_state_.dq.data());
		//Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> u_now_(robot_state_.tau_J.data());

		//get upsampled solution trajectory, at hardware frequency 1kHz 
		executor_sub_ = node_handle.subscribe("/upsampled_sol_traj", 1, &LinearMPCController::executor_callback, this);
		mpc_t_start_pub_ = node_handle.advertise<std_msgs::Time>("/mpc_t_start", 1, true); //True for latched publisher

		u_cmd_ = Eigen::VectorXd::Zero(NUM_JOINTS);

		ROS_INFO("\n Linear MPC Controller initialized successfully. xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx \n");
        return true;
    }

	//#######################################################################################
    void LinearMPCController::starting(const ros::Time& time) 
    {
		ROS_INFO("Waiting for gazebo /clock to start...");
		/* If simulated time (/clock) is OFF:
		   ros::Time::now() returns wall-clock time since Unix epoch (e.g. seconds since 1970). 
		   So ros::Time::now().toSec() will be a large positive number, definitely never zero.

		   If simulated time (/clock) is ON:
		   ROS time is controlled by the /clock topic, which may start at zero. Until /clock publishes, 
		   ros::Time::now() returns zero (or the zero time). So checking ros::Time::now().toSec() == 0.0 
		   is valid only if you expect simulated time and want to wait for it to start.*/
		while (ros::Time::now().toSec() == 0.0 && ros::ok()) 
		{
			ros::Duration(0.1).sleep();
		}
		ROS_INFO("Clock started at %f", ros::Time::now().toSec());

		// publish mpc starting time
		mpc_t_start_msg_.data = ros::Time::now();
		mpc_t_start_pub_.publish(mpc_t_start_msg_);
		ROS_INFO_STREAM("Published latched /mpc_t_start = " << mpc_t_start_msg_.data.toSec());


        // set the condition here to check if current robot state is close to x0_ref
		if (!initial_pose_ok())
		{
			//go_to_init_pose(xref_now_.col(0));
		}

		std::cout << '\n' << std::endl;
		ROS_INFO("checking if initial pose is ok inside starting() \n");

		// // press enter to continue
		// std::cout << "Press Enter to start MPC controller..." << std::endl;
		// std::cin.get();

    }

	//#######################################################################################
    void LinearMPCController::update(const ros::Time& time, const ros::Duration& period) 
	{
		// joint_state_msg_copy_ = latest_joint_state_msg_;
		// joint_state_msg_copy_.header.stamp = ros::Time::now();

		//// press enter to continue
		//std::cout << "Press Enter to start MPC controller..." << std::endl;
		//std::cin.get();

		robot_state_ = state_handle_->getRobotState();
		q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(robot_state_.q.data());
		v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(robot_state_.dq.data());
		u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(robot_state_.tau_J.data());
		

		// IN: set the torques to the robot, obtained from mpc_solver_node
		for (size_t i = 0; i < NUM_JOINTS; i++) {
			// ROS_INFO("checking inside update(), 2, in the for loop \n");
			//std::cout << "u_cmd_: " << u_cmd_.transpose() << std::endl;
			joint_handles_[i].setCommand(u_cmd_[i]); //u_cmd_ is from the sub
			// ROS_INFO("checking inside update(), 3, in the for loop \n");
		}

    }

	//#######################################################################################
	void LinearMPCController::joint_state_callback_sim(const sensor_msgs::JointState::ConstPtr& msg) 
	{
		//store current state
		// get the current joint position and velocity
		//std::cout << '\n' << std::endl;
		//std::lock_guard<std::mutex> lock(joint_state_mutex_);
		//latest_joint_state_msg_ = *msg;
		//q_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->position.data());
		//v_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->velocity.data());
		//u_now_ = Eigen::Map<const Eigen::Matrix<double, NUM_JOINTS, 1>>(msg->effort.data());
	}

	//#######################################################################################
	bool LinearMPCController::initial_pose_ok()
	{
		return true;
	}

	//#######################################################################################
	void LinearMPCController::executor_callback(const std_msgs::Float64MultiArray::ConstPtr& dim7_vec_msg) 
	{
		
		// ROS_INFO("checking inside executor_callback(), 1 &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& \n");
		//get the upsampled solution
		if (dim7_vec_msg->data.size() == 0)
		{
			return;
		}
		else
		{
			u_cmd_ = Eigen::Map<const Eigen::VectorXd>(dim7_vec_msg->data.data(), dim7_vec_msg->data.size());
		}
	}


} //namespace linearmpc_panda

PLUGINLIB_EXPORT_CLASS(linearmpc_panda::LinearMPCController, controller_interface::ControllerBase)

