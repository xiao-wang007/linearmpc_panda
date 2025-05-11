#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("example_topic", 10);

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello, ROS!";
        pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}