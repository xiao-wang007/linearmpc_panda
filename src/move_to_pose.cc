#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <Eigen/Dense>

//###############################################################################################
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "move_to_pose");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // Get the pose case from parameter (default: "default")
  std::string pose_case;
  nh_priv.param<std::string>("case", pose_case, "default");
  ROS_INFO("Loading pose for case: %s", pose_case.c_str());

  // Load q_desired from parameter server based on case
  std::string param_name = "init_poses/" + pose_case;
  std::vector<double> q_desired_std;
  if (!nh_priv.getParam(param_name, q_desired_std)) {
    ROS_ERROR("Missing parameter '%s'. Available cases should be defined in init_poses.yaml", param_name.c_str());
    ROS_ERROR("Usage: rosrun linearmpc_panda move_to_pose _case:=<case_name>");
    return 1;
  }
  
  if (q_desired_std.size() != 7) {
    ROS_ERROR("q_desired for case '%s' must have 7 elements, got %zu", pose_case.c_str(), q_desired_std.size());
    return 1;
  }
  
  Eigen::Map<Eigen::VectorXd> q_desired(q_desired_std.data(), q_desired_std.size());
  ROS_INFO_STREAM("Desired pose for case '" << pose_case << "': " << q_desired.transpose());

  // Get initial joint state
  sensor_msgs::JointState::ConstPtr joint_state_msg =
      ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", ros::Duration(2.0));

  if (!joint_state_msg) {
      ROS_ERROR("Failed to get initial joint state");
      return 1;
  }

  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/position_joint_trajectory_controller/command", 10);

  // Wait for the publisher to connect to subscribers
  ros::Duration(1.0).sleep();

  // convert joint_state_msg to eigen
  Eigen::Map<const Eigen::VectorXd> q_start(joint_state_msg->position.data(), 
                                              joint_state_msg->position.size());
  std::cout << "Panda current pose: \n" << q_start.transpose() << std::endl;
                      
  // Eigen::VectorXd q_desired(7);
  // // q_desired << 0.770901, 0.396021, -0.812618, -2.17939, 0.663888, 2.34041, 0.;

  // // for N=60, Euler, the 1st real exp 
  // q_desired << 0.9207, 0.2574, -0.9527, -2.0683, 0.2799, 2.1147, 2.0;
  
  // // q_desired << 0.485 , -0.2725, -0.6025, -2.5861,  0.4472,  2.1903, -0.4899;

  // /* for exp 10 and 10.5 */
  // // q_desired << 0.485 , -0.2725, -0.6025, -2.5861,  0.4472,  2.1903, -0.4899;

  // /* for exp 11 and 11.5 */
  // // q_desired << 0.6465, -0.4952, -0.0675, -2.8818,  0.6238,  2.572 ,  0.22666;



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

  ROS_INFO("Publishing trajectory...");

  traj_pub.publish(traj_msg);

  ROS_INFO("Trajectory complete.");
  return 0;
}
