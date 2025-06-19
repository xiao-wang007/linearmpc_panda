#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <Eigen/Dense>


int main(int argc, char** argv) 
{
  ros::init(argc, argv, "fake_joint_state_publisher");
  ros::NodeHandle nh;


  //sub to get x_init_desired
  ros::Publisher x_init_desired_pub = nh.advertise<sensor_msgs::JointState>("/franka_state_controller/joint_states", 1, true);

  Eigen::VectorXd q(7);
  q << 0.770901, 0.396021, -0.812618, -2.17939, 0.663888, 2.34041, 0.;

  Eigen::VectorXd v = Eigen::VectorXd::Zero(7);

  sensor_msgs::JointState fake_state_msg;
  fake_state_msg.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", 
                         "panda_joint5", "panda_joint6", "panda_joint7"};
  fake_state_msg.position = std::vector<double>(q.data(), q.data() + q.size());
  fake_state_msg.velocity = std::vector<double>(v.data(), v.data() + v.size());

  ROS_INFO("fake joint state publisher started!");

  while (ros::ok()) 
  {
    // Publish the fake joint state at a fixed rate
    x_init_desired_pub.publish(fake_state_msg);
    ros::spinOnce();
    ros::Duration(0.001).sleep(); // Sleep for 1 ms
  }

  return 0;
}