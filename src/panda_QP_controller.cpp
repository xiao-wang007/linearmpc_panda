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

    void QPController::starting(const ros::Time & /*time*/) {

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
} //namespace linearmpc_panda

PLUGINLIB_EXPORT_CLASS(linearmpc_panda::QPController, controller_interface::ControllerBase)

