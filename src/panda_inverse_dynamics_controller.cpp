#include "panda_inverse_dynamics_controller.h"
#include <pluginlib/class_list_macros.h>
#include <franka/robot_state.h>
#include <franka_hw/franka_model_interface.h>
#include <hardware_interface/hardware_interface.h>

namespace panda_inverse_dynamics_controller {
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

        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();

        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint1"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint2"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint3"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint4"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint5"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint6"));
        joint_handles_.push_back(effort_joint_interface->getHandle("panda_joint7"));

        // Create subscriber for user to send messages to the controller
        state_subscriber_ = node_handle.subscribe("/desired_state", 10, &InverseDynamicsController::desiredStateCallback, this);

        desired_positions_.resize(NUM_JOINTS, 0.0);
        desired_velocities_.resize(NUM_JOINTS, 0.0);

        // Load gains from parameter server
        node_handle.getParam("p_gains", k_p_);
        node_handle.getParam("d_gains", k_d_);

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

        // Compute Mass, Coriolis and Gravity matrices
        std::array<double, NUM_JOINTS*NUM_JOINTS> mass_matrix = model_handle_->getMass();
        std::array<double, NUM_JOINTS>            coriolis    = model_handle_->getCoriolis();
        std::array<double, NUM_JOINTS>            gravity     = model_handle_->getGravity();

        // Map mass, coriolis an gravity to Eigen matrices
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, NUM_JOINTS, Eigen::RowMajor>> M(mass_matrix.data());
        Eigen::Map<Eigen::Matrix<double, NUM_JOINTS, 1>> C(coriolis.data());


        // Compute the desired torques (tau = M*qdd_desired + C*qg + g)
        // IMPORTANT NOTE: No need to add gravity here, as gravity is automatically added by the Franka robot
        // Also franka automatically multiplies coriolis matrix by current velocity vector for us.
        Eigen::Matrix<double, NUM_JOINTS, 1> tau_desired = (M * qdd_desired) + C;

        // Apply torques to the robot
        std::array<double, NUM_JOINTS> tau_command;
        for (size_t i = 0; i < NUM_JOINTS; i++) {
            tau_command[i] = tau_desired(i);
        }

        ROS_INFO("commanded torque: %f, %f, %f, %f, %f, %f, %f",
                 tau_command[0], tau_command[1], tau_command[2], tau_command[3], tau_command[4], tau_command[5], tau_command[6]);

        for (size_t i = 0; i < NUM_JOINTS; i++) {
            joint_handles_[i].setCommand(tau_command[i]);
        }

    }

    void InverseDynamicsController::desiredStateCallback(const panda_inverse_dynamics_controller::DesiredState::ConstPtr& msg) {
        // TODO - Would be good to impose safety limits on commanded velocity
        // as well as keeping commanded position within joint limits
        for(size_t i = 0; i < NUM_JOINTS; i++) {
            desired_positions_[i] = msg->positions[i];
            desired_velocities_[i] = msg->velocities[i];
        }
    }
} //namespace panda_inverse_dynamics_controller

PLUGINLIB_EXPORT_CLASS(panda_inverse_dynamics_controller::InverseDynamicsController, controller_interface::ControllerBase)
