#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <Eigen/Dense>
#include <std_srvs/Trigger.h>

//global variable
Eigen::VectorXd latest_q_init_desired(7);
bool q_init_desired_received = false;
// Publisher must be global to use in service callback
ros::Publisher traj_pub;

//###############################################################################################
//sub callback to fetch the q_init_desired
void q_init_desired_callback(const sensor_msgs::JointState::ConstPtr& msg) 
{
  if (msg->position.size() == 7) 
  {
    latest_q_init_desired = Eigen::Map<const Eigen::VectorXd>(msg->position.data(), msg->position.size());
    q_init_desired_received = true;
    ROS_INFO("Received initial desired joint positions: \n %s", latest_q_init_desired.transpose().c_str());
  } else {
    ROS_WARN("Received invalid joint positions data size: %zu", msg->data.size());
  }
}

//###############################################################################################
bool move_to_pose_service(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) 
{
  if (!q_desired_received) 
  {
    res.success = false;
    res.message = "No desired q received yet.";
    return true;
  }

  // Get initial joint state
  sensor_msgs::JointState::ConstPtr joint_state_msg =
      ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(2.0));
  
  if (!joint_state_msg) {
      res.success = false;
      res.message = "Failed to get initial joint state";
      return true;
  }

  // convert joint_state_msg to eigen
  Eigen::Map<const Eigen::VectorXd> q_start(joint_state_msg->position.data(), 
                                              joint_state_msg->position.size());
  std::cout << "Panda current pose: \n" << q_start.transpose() << std::endl;
  
  Eigen::VectorXd q_desired = latest_q_init_desired;

  // breaks
  double t_end = 4.;
  Eigen::VectorXd t_breaks(2); 
  t_breaks << 0., t_end;
  
  // stack sample together
  Eigen::MatrixXd samples (q_desired.size(), 2);
  samples.col(0) = q_start;
  samples.col(1) = q_desired;

  auto dq_start = Eigen::VectorXd::Zero(q_start.size());
  auto dq_desired = Eigen::VectorXd::Zero(q_start.size());
  auto cubic_spline = drake::trajectories::PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(t_breaks, samples, dq_start, dq_desired);

  // build the msg
  int nTimes = 20;
  int t_start = 0.0;
  auto ts = Eigen::VectorXd::LinSpaced(nTimes, t_start, t_end);

  std::cout << "ts: " << ts.transpose() << std::endl;

  trajectory_msgs::JointTrajectory traj_msg;
  traj_msg.joint_names = {"panda_joint1", "panda_joint2", "panda_joint3",
                          "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  trajectory_msgs::JointTrajectoryPoint pose_i;

  Eigen::VectorXd q_temp(7);
  Eigen::VectorXd v_temp(7);
  for (int i = 0; i < ts.size(); i++)
  {
    q_temp = cubic_spline.value(ts(i));
    v_temp = cubic_spline.derivative(1).value(ts(i));

    pose_i.positions = std::vector<double>(q_temp.data(), q_temp.data() + q_temp.size()); 
    pose_i.velocities = std::vector<double>(v_temp.data(), v_temp.data() + v_temp.size());
    pose_i.time_from_start = ros::Duration(ts(i));
    traj_msg.points.push_back(pose_i);
  }
  
  traj_pub.publish(traj_msg);
  res.success = true;
  res.message = "Trajectory sent successfully.";

  return true;
}


//###############################################################################################
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "move_to_pose");
  ros::NodeHandle nh;

  //sub to get q_init_desired
  ros::Subscriber q_init_desired_sub = nh.subscribe("/q_init_desired", 10, q_init_desired_callback);

  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/position_joint_trajectory_controller/command", 10);

  //advertise the service
  ros::ServiceServer this_service = nh.advertiseService("move_to_pose", move_to_pose_service);
  //q_desired << 0.770901, 0.396021, -0.812618, -2.17939, 0.663888, 2.34041, -0.5;

  ROS_INFO("move_to_pose service ready.");
  ros::spin();

  return 0;
}
