#include <linearmpc_panda/fake_state_publisher.h>

FakeStatePublisherNode::FakeStatePublisherNode()
{
    // Initialize the node handle
    //nh_ = ros::NodeHandle("~");
    nh_ = ros::NodeHandle();

    // Get parameters from the parameter server
    nh_.param("frequency", frequency_, 1000.0);
    nh_.param("joint_positions", joint_positions_, std::vector<double>{0.770901, 0.396021, -0.812618, -2.17939, 0.663888, 2.34041, -0.5});
    nh_.param("joint_velocities", joint_velocities_, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Initialize the publisher
    fake_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

    // Initialize the timer
    timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &FakeStatePublisherNode::publishFakeState, this);


}

void FakeStatePublisherNode::build_simulation()
{
    drake::systems::DiagramBuilder<double> builder;
    auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(&builder, h_sim_);
    drake::multibody::Parser(&plant).AddModelsFromUrl(pandanogripperfile_);
    const auto& arm_base_frame = plant.GetFrameByName("panda_link0");
    plant.WeldFrames(plant.world_frame(), arm_base_frame, X_W_base_);
    plant.Finalize();

    auto diagram = builder.Build();

    

}

    //
void FakeStatePublisherNode::publishFakeState(const ros::TimerEvent& event)
{
    fake_state_msg_.header.stamp = ros::Time::now();
    fake_state_msg_.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
    fake_state_msg_.position = joint_positions_;
    fake_state_msg_.velocity = joint_velocities_;
    fake_state_msg_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Publish the message
    fake_state_pub_.publish(fake_state_msg_);
    ROS_INFO("Fake joint state published: %f", fake_state_msg_.header.stamp.toSec());
}


void FakeStatePublisherNode::run()
{
    // Spin the node
    ros::spin();
}

// Main function
int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "fake_state_publisher");

    // Create an instance of the FakeStatePublisherNode
    FakeStatePublisherNode fake_state_publisher_node;

    // Run the node
    fake_state_publisher_node.run();

    return 0;
}