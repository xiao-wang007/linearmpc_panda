#include "mpc_executor_node.h"

MPCExecutorNode::MPCExecutorNode()
    : solver_frequency_(25.0), // MPC solver runs at 25 Hz
      executor_frequency_(1000.0), // Executor runs at 1 kHz
      nu_(7) // Number of control inputs
{
    // Subscribe to the MPC solution topic
    mpc_sol_sub_ = nh_.subscribe("mpc_sol", 1, &MPCExecutorNode::mpc_sol_callback, this);

    // Publisher for the upsampled control commands
    u_cmd_upsampled_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("upsampled_sol_traj", 1);

    // Timer for publishing upsampled commands
    upsample_timer_ = nh_.createTimer(ros::Duration(1.0 / executor_frequency_),
                                      &MPCExecutorNode::publish_upsampled_command, this);
}

void MPCExecutorNode::mpc_sol_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) 
{
    // Extract the MPC solution from the message
    int rows = msg->layout.dim[0].size;
    int cols = msg->layout.dim[1].size;

    Eigen::MatrixXd u_ref = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg->data.data(), rows, cols);

    // Create time points for the spline
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(cols, 0.0, cols / solver_frequency_);

    // Create a PiecewisePolynomial spline for the control inputs
    u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(times, u_ref);
}

void MPCExecutorNode::publish_upsampled_command(const ros::TimerEvent& event) 
{
    if (u_cmd_spline_.empty()) {
        ROS_WARN_THROTTLE(1.0, "No spline data available yet.");
        return;
    }

    // Get the current time relative to the spline
    double t_now = ros::Time::now().toSec();
    double t_spline = fmod(t_now, u_cmd_spline_.end_time());

    // Evaluate the spline to get the upsampled control command
    u_cmd_now_ = u_cmd_spline_.value(t_spline);

    // Publish the upsampled command
    upsampled_msg.data.resize(u_cmd_now_.size());
    Eigen::VectorXd::Map(upsampled_msg.data.data(), u_cmd_now_.size()) = u_cmd_now_;

    upsampled_cmd_pub_.publish(upsampled_msg);
}

void MPCExecutorNode::run() {
    ros::spin();
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mpc_executor_node");
    MPCExecutorNode executor_node;
    executor_node.run();
    return 0;
}