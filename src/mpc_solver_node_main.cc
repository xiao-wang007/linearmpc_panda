#include "mpc_solver_node.h"

int main(int argc, char** argv) 
{
    // Initialize the ROS node
    ros::init(argc, argv, "mpc_solver_node");
    
    // Create an instance of the MPCSolverNode class
    MPCSolverNode mpc_solver_node;
    
    // Run the node
    mpc_solver_node.run();
    
    return 0;
}