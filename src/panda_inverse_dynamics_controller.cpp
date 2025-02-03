#include "panda_inverse_dynamics_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/hardware_interface.h>

namespace panda_inverse_dynamics {
    bool InverseDynamicsController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) {
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
        effort_interface_ = std::make_unique<hardware_interface::EffortJointInterface>(*effort_interface);

//        // Subscribe to desired torques topic
//        torque_subscriber_ = node_handle.subscribe("/desired_torques", 10,
//                                                   &InverseDynamicsController::desiredTorqueCallback, this);

        desired_positions_.resize(NUM_JOINTS, 0.0);
        desired_velocities_.resize(NUM_JOINTS, 0.0);

        // TODO - experiment with gain values
//        k_p_ = {400.0, 400.0, 400.0, 400.0, 250.0, 150.0, 100.0};  // Proportional gains
//        k_d_ = {40.0, 40.0, 40.0, 40.0, 30.0, 20.0, 10.0};
        k_p_ = {400.0, 400.0, 400.0, 400.0, 250.0, 150.0, 100.0};  // Proportional gains
        k_d_ = {40.0, 40.0, 40.0, 40.0, 30.0, 20.0, 10.0};

        return true;
    }

    void InverseDynamicsController::starting(const ros::Time & /*time*/) {
        // When controller is started, initialise desired positions to current configuration
        // of the arm, and desired velocity to zero
        franka::RobotState robot_state = state_handle_->getRobotState();
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            desired_positions_[i] = robot_state.q[i];
            desired_velocities_[i] = 0.0;
        }
    }

    void InverseDynamicsController::update(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
        franka::RobotState robot_state = state_handle_->getRobotState();

        // Convert current robot position and velocity into Eigen data storage
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> dq(robot_state.dq.data());

        // Convert desired states to Eigen
        Eigen::Matrix<double, NUM_JOINTS, 1> q_desired, dq_desired;
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            q_desired(i) = desired_positions_[i];
            dq_desired(i) = desired_velocities_[i];
        }

        // Compute desired acceleration from PD control
        Eigen::Matrix<double, NUM_JOINTS, 1> qdd_desired;
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            qdd_desired(i) = k_p_[i] * (q_desired(i) - q(i)) + k_d_[i] * (dq_desired(i) - dq(i));
        }

//        // Compute desired acceleration from PD control
//        std::array<double, NUM_JOINTS> desired_accelerations;
//        for (size_t i = 0; i < NUM_JOINTS; i++) {
//            desired_accelerations[i] = k_p_[i] * (desired_positions_[i] - robot_state.q[i]) +
//                                       k_d_[i] * (desired_velocities_[i] - robot_state.dq[i]);
//        }

        // Compute Mass, Coriolis and Gravity matrices
        std::array<double, NUM_JOINTS*NUM_JOINTS> mass_matrix = model_handle_->getMass();
        std::array<double, NUM_JOINTS>            gravity     = model_handle_->getGravity();
        std::array<double, NUM_JOINTS>            coriolis    = model_handle_->getCoriolis();

        // Map mass, coriolis an gravity to Eigen matrices
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS>> M(mass_matrix.data());
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> C(coriolis.data());
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> G(gravity.data());

        // Compute the desired torques
        Eigen::Matrix<double, NUM_JOINTS, 1> tau_desired = M * qdd_desired + C + G;

        // Apply torques to the robot
        std::array<double, NUM_JOINTS> tau_command;
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            tau_command[i] = tau_desired(i);
        }

        effort_interface_->getHandle("panda_joint1").setCommand(tau_command[0]);
        effort_interface_->getHandle("panda_joint2").setCommand(tau_command[1]);
        effort_interface_->getHandle("panda_joint3").setCommand(tau_command[2]);
        effort_interface_->getHandle("panda_joint4").setCommand(tau_command[3]);
        effort_interface_->getHandle("panda_joint5").setCommand(tau_command[4]);
        effort_interface_->getHandle("panda_joint6").setCommand(tau_command[5]);
        effort_interface_->getHandle("panda_joint7").setCommand(tau_command[6]);
    }

    void InverseDynamicsController::desiredStateCallback(const panda_inverse_dynamics_controller::DesiredState::ConstPtr& msg) {
        if (msg->positions.size() != NUM_JOINTS || msg->velocities.size() != NUM_JOINTS) {
            ROS_WARN("Received incorrect state vector size. Expected 7 positions and 7 velocities.");
            return;
        }

        // TODO - Would be good to impose safety limits on commanded velocity
        // as well as keeping commanded position within joint limits
        for(size_t i = 0; i < NUM_JOINTS; i++) {
            desired_positions_[i] = msg->positions[i];
            desired_velocities_[i] = msg->velocities[i];
        }
    }
} //namespace panda_inverse_dynamics

PLUGINLIB_EXPORT_CLASS(panda_inverse_dynamics::InverseDynamicsController, controller_interface::ControllerBase)
