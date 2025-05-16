#pragma once

#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Core>
#include <linearmpc_panda/DesiredTorqueState.h>

namespace linearmpc_panda{
    #define NUM_JOINTS 7

    class TorquePDController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            franka_hw::FrankaStateInterface,
            hardware_interface::EffortJointInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;

        void update(const ros::Time &time, const ros::Duration &period) override;

        void starting(const ros::Time &time) override;

    private:
        void desiredTorqueStateCallback(const linearmpc_panda::DesiredTorqueState::ConstPtr& msg);

        void saturateTorqueRate( std::array<double, 7>& tau_d_command,
                                 const std::array<double, 7>& tau_J_d);

        std::array<double, 7> last_sent_torques_{};

        ros::Subscriber state_subscriber_;

        std::unique_ptr <franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr <franka_hw::FrankaStateHandle> state_handle_;

        std::vector<hardware_interface::JointHandle> joint_handles_;

        std::vector<double> desired_torques_;
        std::vector<double> desired_velocities_;
        std::vector<double> desired_positions_;
        std::vector<double> k_p_;  // Proportional gains
        std::vector<double> k_d_;  // Derivative gains

        double alpha_;  // Low pass filter rate
        bool torque_rate_limiter_;
        bool low_pass_filter_;

        const double kdeltaTauMax_ = 1.0;

    };
} // namespace linearmpc_panda