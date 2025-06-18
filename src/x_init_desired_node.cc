#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
//#include "../include/linearmpc_panda/myutils.h"
#include <linearmpc_panda/myutils.h>


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "x_init_desired_node");
  ros::NodeHandle nh;

  int N_ = 150;
  std::vector<std::string> var_names_ = {"q_panda", "v_panda", "a_panda", "us", "h"};
  std::vector<int> dims_ = {7, 7, 7, 7, 1};
  std::vector<int> times_ = {N_, N_, N_, N_, N_-1};
  //std::string ref_traj_path_ = "/home/rosdrake/src/src/mpc/traj_refs/joint7_traj_N60_tf1.2.npy";
  std::string ref_traj_path_ = "/home/rosdrake/src/src/mpc/traj_refs/joint7_traj_N150_tf3.0.npy";

  MyUtils::ProcessedSolution data_proc_; 
  data_proc_ = MyUtils::ProcessSolTraj(ref_traj_path_, var_names_, dims_, times_);

  //sub to get x_init_desired
  ros::Publisher x_init_desired_pub = nh.advertise<sensor_msgs::JointState>("/x_init_desired", 1, true);

  Eigen::VectorXd q_init_desired = data_proc_.trajs.at("q_panda").row(0); 
  Eigen::VectorXd v_init_desired = data_proc_.trajs.at("v_panda").row(0);

  sensor_msgs::JointState x_init_desired_msg;
  x_init_desired_msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", 
                             "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
  x_init_desired_msg.position = std::vector<double>(q_init_desired.data(), q_init_desired.data() + q_init_desired.size());
  x_init_desired_msg.velocity = std::vector<double>(v_init_desired.data(), v_init_desired.data() + v_init_desired.size());

  x_init_desired_pub.publish(x_init_desired_msg);
  ROS_INFO("x_init_desired published!");
  ros::spin();

  return 0;
}