#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

void callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    
    // ROS_INFO("Received vector:");
    std::cout << "upsampled u_cmd: [" << std::endl;

    for (size_t i = 0; i < msg->data.size(); ++i)
    {
        std::cout <<  msg->data[i] << ", ";
    }
    std::cout << "]" << std::endl;
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "upsampled_ucmd_sub_test_node");
  ros::NodeHandle nh;

  ros::Subscriber u_cmd_sub = nh.subscribe("/upsampled_u_cmd", 1, callback);

  ROS_INFO("Upsampled u_cmd node started.");
  ros::spin();
}