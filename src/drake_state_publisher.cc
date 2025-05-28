#include <linearmpc_panda/drake_state_publisher.h>

DrakeStatePublisherNode::DrakeStatePublisherNode()
{
    // Initialize the node handle
    //nh_ = ros::NodeHandle("~");
    nh_ = ros::NodeHandle();

    // Get parameters from the parameter server
    nh_.param("frequency", frequency_, 1000.0);
    nh_.param("init_q", joint_positions_, std::vector<double>{0.770901, 0.396021, -0.812618, -2.17939, 0.663888, 2.34041, -0.5});
    nh_.param("init_v", joint_velocities_, std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Initialize the publisher
    state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1);
    control_sub_ = nh_.subscribe("/upsampled_u_cmd", 1, &DrakeStatePublisherNode::control_callback, this);

    // Initialize the timer
    // timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_), &DrakeStatePublisherNode::publish_state, this);

    std::cout << "checking in DrakeStatePublisherNode constructor" << std::endl;

    build_simulation();
}

//################################################################################################################
void DrakeStatePublisherNode::control_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(control_mutex_);
    latest_control_input_ = Eigen::Map<const Eigen::VectorXd>(msg->data.data(), msg->data.size());
    // std::cout << "control input in callback: " << latest_control_input_.transpose() << std::endl;
}

//################################################################################################################
void DrakeStatePublisherNode::build_simulation()
{
    drake::systems::DiagramBuilder<double> builder;
    plant_ptr_ = builder.AddSystem<drake::multibody::MultibodyPlant<double>>(h_sim_);
    scene_graph_ = builder.AddSystem<drake::geometry::SceneGraph<double>>();

    std::cout << "checking in DrakeStatePublisherNode build_simulation() 1" << std::endl;

    // plant_ptr_ = std::make_unique<drake::multibody::MultibodyPlant<double>>(h_sim_);
    // scene_graph_ = std::make_unique<drake::geometry::SceneGraph<double>>();
    plant_ptr_->RegisterAsSourceForSceneGraph(scene_graph_);

    drake::multibody::Parser parser(plant_ptr_); // .get() is a method of std::unique_ptr
    parser.AddModels(pandanogripperfile_);
    const auto& arm_base_frame = plant_ptr_->GetFrameByName("panda_link0");
    plant_ptr_->WeldFrames(plant_ptr_->world_frame(), arm_base_frame, X_W_base_);
    plant_ptr_->Finalize();
    std::cout << "checking in DrakeStatePublisherNode build_simulation() 2" << std::endl;

    // Connect geometry ports
    builder.Connect(
        plant_ptr_->get_geometry_poses_output_port(),
        scene_graph_->get_source_pose_port(plant_ptr_->get_source_id().value()));

    builder.Connect(
        scene_graph_->get_query_output_port(),
        plant_ptr_->get_geometry_query_input_port());

    diagram_ = builder.Build();
    auto diagram_context_ = diagram_->CreateDefaultContext();
    diagram_->SetDefaultContext(diagram_context_.get()); // seems unnecessary
    // plant_context_ = diagram_->GetMutableSubsystemContext(*(plant_ptr_.get()), diagram_context_.get()).Clone(); 
    // plant_context_ = &diagram_->GetMutableSubsystemContext(*(plant_ptr_.get()), diagram_context_.get()); // another option

    std::cout << "checking in DrakeStatePublisherNode build_simulation() 3" << std::endl;

    simulator_ = std::make_unique<drake::systems::Simulator<double>>(*diagram_, std::move(diagram_context_));
    std::cout << "checking in DrakeStatePublisherNode build_simulation() 4" << std::endl;
    simulator_.get()->set_publish_every_time_step(true);
    simulator_.get()->set_target_realtime_rate(1.); // =1. means =real-time, > 1. means faster than real-time
    simulator_.get()->Initialize();
    // simulator_.AdvanceTo(2.);

    std::cout << "checking in DrakeStatePublisherNode build_simulation() 5" << std::endl;
    set_initial_state();

    ROS_INFO("Simulation built successfully!");

}

//################################################################################################################
void DrakeStatePublisherNode::publish_state() 
{
    auto& context = diagram_->GetMutableSubsystemContext(
        *(plant_ptr_), &simulator_->get_mutable_context());
    
    Eigen::VectorXd state = plant_ptr_->GetPositionsAndVelocities(context);
    // std::cout << "Publishing state of size: " << state.size() << std::endl;
    // std::cout << "state: " << state.transpose() << std::endl;

    // std::cout << "[drake_sim_node] state: " << state.transpose() << std::endl;
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();

    msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    auto nq = joint_positions_.size();
    auto nv = joint_velocities_.size();
    msg.position.resize(nq);
    msg.velocity.resize(nv);  // optional, empty
    msg.effort = {};    // optional, empty

    for (int i = 0; i < nq; i++) {
        msg.position[i] = state[i];
    }

    for (int i = 0; i < nv; i++) {
        msg.velocity[i] = state[nq + i];
    }
    state_pub_.publish(msg);
}


//################################################################################################################
void DrakeStatePublisherNode::set_initial_state()
{
    std::cout << "checking in DrakeStatePublisherNode set_initial_state() 1" << std::endl;
    auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_ptr_, &simulator_->get_mutable_context());
    std::cout << "checking in DrakeStatePublisherNode set_initial_state() 2" << std::endl;

    // Set the initial state of the plant
    Eigen::VectorXd init_state(joint_positions_.size() + joint_velocities_.size());
    init_state <<  Eigen::Map<const Eigen::VectorXd>(joint_positions_.data(), joint_positions_.size()),
                  Eigen::Map<const Eigen::VectorXd>(joint_velocities_.data(), joint_velocities_.size());
    std::cout << "checking in DrakeStatePublisherNode set_initial_state() 3" << std::endl;
    plant_ptr_->SetPositionsAndVelocities(&plant_context, init_state);
    std::cout << "checking in DrakeStatePublisherNode set_initial_state() 4" << std::endl;
}

//################################################################################################################
void DrakeStatePublisherNode::run()
{
    nh_.setParam("/simulation_ready", true);  // Global flag
    ROS_INFO("Simulation node is ready!");

    ros::Rate rate(frequency_);
    // Spin the node
    while (ros::ok())
    {
        if (!plant_ptr_)
        { }

        auto& sim_context = simulator_->get_mutable_context();
        auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_ptr_, &sim_context);

        // Apply control (thread-safe)
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (latest_control_input_.size() == plant_ptr_->num_actuators()) 
            {
                // std::cout << "Applying control input: " << latest_control_input_.transpose() << std::endl;
                plant_ptr_->get_actuation_input_port().FixValue(
                    &plant_context, latest_control_input_);
            }
        }

        // Advance the simulation
        simulator_->AdvanceTo(simulator_->get_context().get_time() + h_sim_);

        // publish the state
        publish_state();
        ros::spinOnce(); // this has to be in the while loop to ensure pubs and subs are processed

        // Sleep to maintain the desired frequency
        ros::Duration(1.0 / frequency_).sleep();
    }
}

//################################################################################################################
// Main function
int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "drake_state_publisher");

    // Create an instance of the DrakeStatePublisherNode
    DrakeStatePublisherNode drake_state_publisher_node;

    // Run the node
    drake_state_publisher_node.run();

    return 0;
}