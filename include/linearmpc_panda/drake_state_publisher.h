#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <string>
#include <linearmpc_panda/myutils.h>
#include <vector>
#include "drake/systems/analysis/simulator.h" 
#include <std_msgs/Float64MultiArray.h>

#define PI 3.14159265358979323846

using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::Vector3;

class DrakeStatePublisherNode
{
public:
    DrakeStatePublisherNode();
    void run();
    void publish_state();
    void build_simulation();
    void set_initial_state();
    void control_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    ~DrakeStatePublisherNode() = default;

private:
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Subscriber control_sub_;
    ros::Timer timer_;
    sensor_msgs::JointState state_msg_;
    double frequency_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    double h_sim_ {0.001};
    // const std::string pandanogripperfile_ = "/Users/xiao/0_codes/ICBM_drake/drake_models/franka_description/urdf/panda_arm.urdf";
    const std::string pandanogripperfile_ = "/home/rosdrake/drake_models/franka_description/urdf/panda_arm.urdf";
    drake::math::RigidTransform<double> X_W_base_ = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * PI / 180.),
                                                                           Vector3<double>(0., -0.2, 0.));
    
    drake::multibody::MultibodyPlant<double>* plant_ptr_ {};
    drake::geometry::SceneGraph<double>* scene_graph_ {};
    std::unique_ptr<drake::systems::Diagram<double>> diagram_ {};
    // std::unique_ptr<drake::systems::Context<double>> diagram_context_{};  
    // std::unique_ptr<drake::systems::Context<double>> plant_context_ {}; 
    std::unique_ptr<drake::systems::Simulator<double>> simulator_; // 
    // drake::systems::Context<double>* plant_context_ {}; //another option

    Eigen::VectorXd latest_control_input_ {}; // Assuming 7 joints for the Panda robot
    std::mutex control_mutex_;
    std::atomic<bool> received_first_control_ {false}; // Flag to indicate if the simulation is ready
};