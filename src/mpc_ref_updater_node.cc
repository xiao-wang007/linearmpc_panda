#include <linearmpc_panda/mpc_ref_updater_node.h>

namespace MPCControllers
{
}


//########################################################################################
int main(int argc, char** argv) 
{
    // Initialize the ROS node
    ros::init(argc, argv, "mpc_ref_updater_node");
    
    // Create an instance of the MPCSolverNode class
    MPCControllers::MPCRefUpdaterNode mpc_ref_updater_node;
    
    // Run the node
    mpc_ref_updater_node.run();
    
    return 0;
}