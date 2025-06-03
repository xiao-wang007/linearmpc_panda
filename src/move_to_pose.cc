#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "move_to_pose");
  ros::NodeHandle nh;

  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/position_joint_trajectory_controller/command", 10);

  // Wait for the publisher to connect to subscribers
  ros::Duration(1.0).sleep();

  trajectory_msgs::JointTrajectory traj_msg;

  traj_msg.joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = {0.770901, 0.396021, -0.812618, -2.17939, 0.663888, 2.34041, -0.5};  
  point.time_from_start = ros::Duration(3.0);  // Time to reach the pose

  traj_msg.points.push_back(point);

  ROS_INFO("Publishing trajectory...");
  traj_pub.publish(traj_msg);

  ros::Duration(4.0).sleep();  // Wait for the robot to finish motion

  ROS_INFO("Trajectory complete.");
  return 0;
}
