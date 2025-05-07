#include "panda_torque_PD_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/hardware_interface.h>

namespace linearmpc_panda {
    bool TorquePDController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) {
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
        state_subscriber_ = node_handle.subscribe("/desired_torque_state", 10, &TorquePDController::desiredTorqueStateCallback, this);

        desired_positions_.resize(NUM_JOINTS, 0.0);
        desired_velocities_.resize(NUM_JOINTS, 0.0);
        desired_torques_.resize(NUM_JOINTS, 0.0);

        // Load gains from parameter server
        node_handle.getParam("p_gains", k_p_);
        node_handle.getParam("d_gains", k_d_);
        node_handle.getParam("alpha", alpha_);

        // Load bool parameters from parameter server
        node_handle.getParam("torque_rate_limiter", torque_rate_limiter_);
        node_handle.getParam("low_pass_filter", low_pass_filter_);

        if(torque_rate_limiter_){
            ROS_INFO("Torque rate limiter enabled");
        } else {
            ROS_INFO("Torque rate limiter disabled");
        }

        if(low_pass_filter_){
            ROS_INFO("Low pass filter enabled");
        } else {
            ROS_INFO("Low pass filter disabled");
        }

        return true;
    }

    void TorquePDController::starting(const ros::Time & /*time*/) {
        // When controller is started, initialise desired positions to current configuration
        // of the arm, and desired velocity to zero, and desired extra torque to zero
        franka::RobotState robot_state = state_handle_->getRobotState();
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            desired_positions_[i] = robot_state.q[i];
            desired_velocities_[i] = 0.0;
            desired_torques_[i] = 0.0;
        }
    }

    void TorquePDController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
        franka::RobotState robot_state = state_handle_->getRobotState();

        // Convert current robot position and velocity into Eigen data storage
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> dq(robot_state.dq.data());

        // Convert desired states to Eigen
        Eigen::Matrix<double, NUM_JOINTS, 1> q_desired, dq_desired, torque_desired;
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            q_desired(i) = desired_positions_[i];
            dq_desired(i) = desired_velocities_[i];
            torque_desired(i) = desired_torques_[i];
        }

        // Compute actual torques to send to robot with PD feedback
        std::array<double, NUM_JOINTS> tau_command;
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            tau_command[i] = torque_desired[i] + (k_p_[i] * (q_desired(i) - q(i))) + (k_d_[i] * (dq_desired(i) - dq(i)));
        }

        // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
        // 1000 * (1 / sampling_time).
        if(torque_rate_limiter_){
            saturateTorqueRate(tau_command, robot_state.tau_J_d);
        }

        // Apply a low pass filter to the torques to avoid discontinuities
        if(low_pass_filter_){
            for (size_t i = 0; i < NUM_JOINTS; i++) {
                tau_command[i] = alpha_ * tau_command[i] + (1 - alpha_) * last_sent_torques_[i];
            }
        }

        last_sent_torques_ = tau_command;

        // Apply torques to the robot
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            joint_handles_[i].setCommand(tau_command[i]);
        }

    }

    void TorquePDController::saturateTorqueRate( std::array<double, 7>& tau_d_command,
                                                 const std::array<double, 7>& tau_J_d) {
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            double difference = tau_d_command[i] - tau_J_d[i];
            tau_d_command[i] = tau_J_d[i] + std::max(std::min(difference, kdeltaTauMax_), -kdeltaTauMax_);
        }
    }

    void TorquePDController::desiredTorqueStateCallback(const linearmpc_panda::DesiredTorqueState::ConstPtr& msg) {
        // TODO - Would be good to impose safety limits on commanded velocity
        // as well as keeping commanded position within joint limits
        for(size_t i = 0; i < NUM_JOINTS; i++) {
            desired_torques_[i] = msg->torques[i];
            desired_positions_[i] = msg->positions[i];
            desired_velocities_[i] = msg->velocities[i];
        }
    }
} //namespace panda_torque_PD_controller

PLUGINLIB_EXPORT_CLASS(linearmpc_panda::TorquePDController, controller_interface::ControllerBase)