#include "panda_QP_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/hardware_interface.h>

namespace linearmpc_panda {
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

        // Create subscriber for user to send messages to the controller
//        state_subscriber_ = node_handle.subscribe("/desired_state", 10, &InverseDynamicsController::desiredStateCallback, this);

        return true;
    }

    void QPController::starting(const ros::Time & /*time*/) 
    {
        // init the plant   
        this->init_plant();

        // read x_ref and u_ref

        // set the condition here to check if current robot state is close to x0_ref

        // index the first horizon

        // init prog

    }

    void QPController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {

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


    void QPController::init_plant() 
    {
        // Initialize the plant here
		plant_ptr_ = std::make_unique<MultibodyPlant<double>>(h_env_);
		Parser parser(plant_ptr_.get());
		parser.AddModels(panda_file_);
		const auto& arm_base_frame = plant_ptr_->GetFrameByName("panda_link0");

        auto X_W_base = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * PI / 180.),
                                      Vector3<double>(0., -0.2, 0.));
		plant_ptr_->WeldFrames(plant_ptr_->world_frame(), arm_base_frame, X_W_base);

		plant_ptr_->Finalize();
		plantAD_ptr_ = System<double>::ToAutoDiffXd( *(this->plant_ptr_) );
		contextAD_ptr_ = plantAD_ptr_->CreateDefaultContext();
		context_ptr_ = plant_ptr_->CreateDefaultContext();
    }

    void QPController::init_prog() 
    {
        // Initialize the prog here
        
    }

} //namespace linearmpc_panda

PLUGINLIB_EXPORT_CLASS(linearmpc_panda::QPController, controller_interface::ControllerBase)

