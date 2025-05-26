#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <string>
#include <linearmpc_panda/myutils.h>
#include <vector>
#include "drake/systems/analysis/simulator.h" 

#define PI 3.14159265358979323846

using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::Vector3;

class FakeStatePublisherNode
{
public:
    FakeStatePublisherNode();
    void run();
    void publish_state(const ros::TimerEvent& event);
    void build_simulation();
    void set_initial_state();
    ~FakeStatePublisherNode() = default;

private:
    ros::NodeHandle nh_;
    ros::Publisher state_pub_;
    ros::Timer timer_;
    sensor_msgs::JointState state_msg_;
    double frequency_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    double h_sim_ {0.001};
    const std::string pandanogripperfile_ = "/Users/xiao/0_codes/ICBM_drake/drake_models/franka_description/urdf/panda_arm.urdf";
    drake::math::RigidTransform<double> X_W_base_ = RigidTransform<double>(RollPitchYaw<double>(Vector3<double>(0., 0., -90.) * PI / 180.),
                                                                           Vector3<double>(0., -0.2, 0.));
    
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_ptr_ {};
    std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ {};
    std::unique_ptr<drake::systems::Diagram<double>> diagram_ {};
    std::unique_ptr<drake::systems::Context<double>> diagram_context_{};  
    std::unique_ptr<drake::systems::Context<double>> plant_context_ {}; 
    std::unique_ptr<drake::systems::Simulator<double>> simulator_; // 
    // drake::systems::Context<double>* plant_context_ {}; //another option
};