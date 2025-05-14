#include "mpc_executor_node.h"

//########################################################################################
MPCExecutorNode::MPCExecutorNode()
{
    // Subscribe to the MPC solution topic
    mpc_sol_sub_ = nh_.subscribe("mpc_sol", 1, &MPCExecutorNode::mpc_sol_callback, this);

    // Publisher for the upsampled control commands
    u_cmd_upsampled_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("upsampled_sol_traj", 1);

    // Timer for publishing upsampled commands
    upsample_timer_ = nh_.createTimer(ros::Duration(1.0 / executor_frequency_), &MPCExecutorNode::publish_upsampled_command, this);

    // init the matrix to hold the solution matrix
    u_sol_ = Eigen::MatrixXd::Zero(nu_, Nh_);

    Nt_ = Nh_ + 1; // Number of time points

    t_init_node_ = ros::Time::now();
}

//########################################################################################
void MPCExecutorNode::mpc_sol_callback(const linearmpc_panda::StampedFloat64MultiArray::ConstPtr& msg) 
{
    // Extract the MPC solution from the message
    rows_ = msg->data.layout.dim[0].size;
    cols_ = msg->data.layout.dim[1].size;

    std::cout << "Received MPC solution with dimensions: " << rows_ << " x " << cols_ << std::endl;

    assert(rows_ == nu_ && cols_ == Nh_ && "MPC solution dimensions mismatch in executor node!");

    std::cout << "msg->data.data: ";
    for (const auto& value : msg->data.data) {
        std::cout << value << " ";
    }

    std::cout << std::endl;
    u_sol_ = Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
        msg->data.data.data(), rows_, cols_);
    
    std::cout << "u_sol_: \n" << u_sol_ << std::endl;

    // Create time points for the spline, using stamped time
    t_stamp_ = msg->header.stamp;
    ROS_INFO_STREAM("Received MPC solution at time: " << t_stamp_.toSec());
    auto t_relative = (t_stamp_ - t_init_node_).toSec();
    Eigen::VectorXd ts = Eigen::VectorXd::LinSpaced(Nh_, t_relative, t_relative + Nh_*h_mpc_);

    // Create a PiecewisePolynomial spline for the control inputs
    u_cmd_spline_ = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(ts, u_sol_);
}

//########################################################################################
void MPCExecutorNode::publish_upsampled_command(const ros::TimerEvent& event) 
{
    //std::cout << "blablablablablablablablablablablablablablablablabla" << std::endl;
    if (u_cmd_spline_.empty()) {
        ROS_WARN_THROTTLE(1.0, "No spline data available yet.");
    //std::cout << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx" << std::endl;
        return;
    }

    // Get the current time relative to the spline
    t_now_ = ros::Time::now().toSec();

    // Evaluate the spline to get the upsampled control command
    u_cmd_now_ = u_cmd_spline_.value(t_now_);

    //ROS_INFO_STREAM("Publishing upsampled command u_cmd_now: " << u_cmd_now_.transpose());

    // Publish the upsampled command
    upsampled_msg_.data.resize(u_cmd_now_.size());
    Eigen::VectorXd::Map(upsampled_msg_.data.data(), u_cmd_now_.size()) = u_cmd_now_;

    u_cmd_upsampled_pub_.publish(upsampled_msg_);
}

//########################################################################################
void MPCExecutorNode::run() {
    ros::spin();
}

//########################################################################################
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "mpc_executor_node");
    MPCExecutorNode executor_node;
    executor_node.run();
    return 0;
}