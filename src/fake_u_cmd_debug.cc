#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "fake_u_cmd_debug_node");
  ros::NodeHandle nh;

  ros::Publisher u_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("/upsampled_u_cmd", 1);

  std_msgs::Float64MultiArray u_cmd_msg;
  u_cmd_msg.data = {12, 12, 12, 12, 12, 12, 12}; 

  ros::Rate rate(1000); // 1000 Hz

  ROS_INFO("Fake u_cmd debug node started. Publishing u_cmd at 1000 Hz...");
  while (ros::ok())
  {
    u_cmd_pub.publish(u_cmd_msg);
    ros::spinOnce();
    rate.sleep();
  }
}