#pragma once

#include <memory>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

class InverseDynamicsController : public controller_interface::MultiInterfaceController<
        franka_hw::FrankaModelInterface,
        franka_hw::FrankaStateInterface,
        franka_hw::FrankaJointEffortInterface> {
public:
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void update(const ros::Time& time, const ros::Duration& period) override;
    void starting(const ros::Time& time) override;

private:
    void desiredTorqueCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);

    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaJointEffortInterface> effort_interface_;

    std::vector<double> desired_torques_;
    ros::Subscriber torque_subscriber_;
};