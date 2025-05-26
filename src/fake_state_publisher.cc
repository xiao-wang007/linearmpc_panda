#include <linearmpc_panda/fake_state_publisher.h>

FakeStatePublisherNode::FakeStatePublisherNode()
{
    // Initialize the node handle
    //nh_ = ros::NodeHandle("~");
    nh_ = ros::NodeHandle();

    // Get parameters from the parameter server
    nh_.param("frequency", frequency_, 1000.0);
    nh_.param("init_q", joint_positions_, std::vector<double>{0.770901, 0.396021, -0.812618, -2.17939, 0.663888, 2.34041, -0.5});
    nh_.param("init_v", joint_velocities_, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Initialize the publisher
    state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

    // Initialize the timer
    // timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &FakeStatePublisherNode::publish_state, this);

    build_simulation();
}

void FakeStatePublisherNode::build_simulation()
{
    drake::systems::DiagramBuilder<double> builder;
    plant_ptr_ = std::make_unique<drake::multibody::MultibodyPlant<double>>(h_sim_);
    scene_graph_ = std::make_unique<drake::geometry::SceneGraph<double>>();
    plant_ptr_->RegisterAsSourceForSceneGraph(scene_graph_.get());

    drake::multibody::Parser parser(plant_ptr_.get()); // .get() is a method of std::unique_ptr
    parser.AddModelsFromUrl(pandanogripperfile_);
    const auto& arm_base_frame = plant_ptr_.get()->GetFrameByName("panda_link0");
    plant_ptr_.get()->WeldFrames(plant_ptr_.get()->world_frame(), arm_base_frame, X_W_base_);
    plant_ptr_.get()->Finalize();

    diagram_ = builder.Build();
    diagram_context_ = diagram_->CreateDefaultContext();
    diagram_->SetDefaultContext(diagram_context_.get()); // seems unnecessary
    plant_context_ = diagram_->GetMutableSubsystemContext(*(plant_ptr_.get()), diagram_context_.get()).Clone(); 
    // plant_context_ = &diagram_->GetMutableSubsystemContext(*(plant_ptr_.get()), diagram_context_.get()); // another option

    set_initial_state();

    // // Get joints so that we can set initial conditions.
    // const PrismaticJoint<double>& cart_slider =
    //     cart_pole.GetJointByName<PrismaticJoint>("CartSlider");
    // const RevoluteJoint<double>& pole_pin =
    //     cart_pole.GetJointByName<RevoluteJoint>("PolePin");


    simulator_ = std::make_unique<drake::systems::Simulator<double>>( *diagram_, std::move(diagram_context_));
    // simulator_.set_publish_every_time_step(false);
    simulator_.get()->set_target_realtime_rate(0.001);
    simulator_.get()->Initialize();
    // simulator_.AdvanceTo(2.);

}

// using timer callback 
// void FakeStatePublisherNode::publish_state(const ros::TimerEvent& event)
// {
//     state_msg_.header.stamp = ros::Time::now();
//     state_msg_.name = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"};
//     state_msg_.position = joint_positions_;
//     state_msg_.velocity = joint_velocities_;
//     state_msg_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//     // Publish the message
//     state_pub_.publish(state_msg_);
//     ROS_INFO("Fake joint state published: %f", state_msg_.header.stamp.toSec());
// }

void FakeStatePublisherNode::publish_state() 
{
    auto& context = diagram_->GetMutableSubsystemContext(
        *plant_, &simulator_->get_mutable_context());
    
    Eigen::VectorXd state = plant_->GetPositionsAndVelocities(context);

    std_msgs::Float64MultiArray msg;
    msg.data.assign(state.data(), state.data() + state.size());
    state_pub_.publish(msg);
}

//
void FakeStatePublisherNode::set_initial_state()
{
    // Set the initial state of the plant
    Eigen::VectorXd init_state(joint_positions_.size() + joint_velocities_.size());
    init_state <<  Eigen::Map<const Eigen::VectorXd>(joint_positions_.data(), joint_positions_.size()),
                  Eigen::Map<const Eigen::VectorXd>(joint_velocities_.data(), joint_velocities_.size());
    plant_ptr_->SetPositionsAndVelocities(plant_context_.get(), init_state);
}

void FakeStatePublisherNode::run()
{
    ros::Rate rate(frequency_);
    // Spin the node
    while (ros::ok())
    {
        // Build the simulation if not already done
        if (!plant_ptr_)
        { }

        plant_ptr_.get()->get_actuation_input_port().set_value();

        // Advance the simulation
        simulator_->AdvanceTo(simulator_->get_context().get_time() + h_sim_);

        // publish the state
        publish_state();

        // Sleep to maintain the desired frequency
        ros::Duration(1.0 / frequency_).sleep();
    }


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