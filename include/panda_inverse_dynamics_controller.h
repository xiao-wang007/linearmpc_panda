#pragma once

#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
//#include <franka_hw/franka_joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>

#include <panda_inverse_dynamics_controller/DesiredState.h>

namespace panda_inverse_dynamics_controller {
    #define NUM_JOINTS 7

    class InverseDynamicsController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            franka_hw::FrankaStateInterface,
            hardware_interface::EffortJointInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

    private:
        void desiredStateCallback(const panda_inverse_dynamics_controller::DesiredState::ConstPtr& msg);

        ros::Subscriber state_subscriber_;

        std::unique_ptr <franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr <franka_hw::FrankaStateHandle> state_handle_;

        std::vector<hardware_interface::JointHandle> joint_handles_;

        std::vector<double> desired_positions_;
        std::vector<double> desired_velocities_;
        std::vector<double> k_p_;  // Proportional gains
        std::vector<double> k_d_;  // Derivative gai

    };
} // namespace panda_inverse_dynamics_controller