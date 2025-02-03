#include "panda_inverse_dynamics_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/hardware_interface.h>

bool InverseDynamicsController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
    // Get Franka model and state interfaces
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    auto* effort_interface = robot_hw->get<franka_hw::FrankaJointEffortInterface>();

    if (!model_interface || !state_interface || !effort_interface) {
        ROS_ERROR("Failed to get required hardware interfaces.");
        return false;
    }

    // Get handles
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle("panda_model"));
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle("panda_robot"));
    effort_interface_ = std::make_unique<franka_hw::FrankaJointEffortInterface>(*effort_interface);

    // Subscribe to desired torques topic
    torque_subscriber_ = node_handle.subscribe("/desired_torques", 10, &InverseDynamicsController::desiredTorqueCallback, this);

    desired_torques_.resize(7, 0.0);
    return true;
}

void InverseDynamicsController::starting(const ros::Time& /*time*/) {
    desired_torques_ = std::vector<double>(7, 0.0);
}

void InverseDynamicsController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis = model_handle_->getCoriolis();

    std::array<double, 7> torques;
    for (size_t i = 0; i < 7; i++) {
        torques[i] = desired_torques_[i] + coriolis[i]; // Feedforward inverse dynamics
    }

    effort_interface_->getHandle("panda_joint1").setCommand(torques[0]);
    effort_interface_->getHandle("panda_joint2").setCommand(torques[1]);
    effort_interface_->getHandle("panda_joint3").setCommand(torques[2]);
    effort_interface_->getHandle("panda_joint4").setCommand(torques[3]);
    effort_interface_->getHandle("panda_joint5").setCommand(torques[4]);
    effort_interface_->getHandle("panda_joint6").setCommand(torques[5]);
    effort_interface_->getHandle("panda_joint7").setCommand(torques[6]);
}

void InverseDynamicsController::desiredTorqueCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() != 7) {
        ROS_WARN("Received incorrect torque vector size. Expected 7.");
        return;
    }
    desired_torques_ = msg->data;
}



PLUGINLIB_EXPORT_CLASS(franka_inverse_dynamics::InverseDynamicsController, controller_interface::ControllerBase)
