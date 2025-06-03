#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "move_to_pose");
  ros::NodeHandle nh;

  ros::Publisher traj_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/bring_to_init_controller/command", 10);

  // Wait for the publisher to connect to subscribers
  ros::Duration(1.0).sleep();

  trajectory_msgs::JointTrajectory traj_msg;

  traj_msg.joint_names = {
      "panda_joint1", "panda_joint2", "panda_joint3",
      "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};

  trajectory_msgs::JointTrajectoryPoint point;
  need to get this!! 
  point.positions = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};  // Desired pose
  point.time_from_start = ros::Duration(3.0);  // Time to reach the pose

  traj_msg.points.push_back(point);

  ROS_INFO("Publishing trajectory...");
  traj_pub.publish(traj_msg);

  ros::Duration(4.0).sleep();  // Wait for the robot to finish motion

  ROS_INFO("Trajectory complete.");
  return 0;
}
