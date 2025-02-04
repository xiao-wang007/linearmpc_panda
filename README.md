# panda_inverse_dynamics_controller

This repository implements an Inverse Dynamics Controller (IDC) for the Franka-Emika Panda
using franka_ros. The controller computes desired joint torques based on feedforward acceleration 
and the discrepancy between actual and desired positions and velocities.

The controller is broken down into the two following stages, firstly compute a desired acceleration
term:

```math
\begin{align}
\ddot{\textbf{q}}_{desired} = \ddot{\textbf{q}}_{feedforward} + \textbf{K}_p(\textbf{q}_{desired} - \textbf{q}_{actual}) + \textbf{K}_d(\dot{\textbf{q}}_{desired} - \dot{\textbf{q}}_{actual})
\end{align}
```

where the feedforward acceleration and desired positions and velocities are the commands sent to the controller. The actual positions
and velocities are obtained from the real robot state. The desired acceleration is used with the robot inverse dynamics formula to compute
torques:

```math
\begin{align}
\tau = M(\textbf{q}_{actual}) \ddot{\textbf{q}}_{desired} + C(\textbf{q}_{actual}, \dot{\textbf{q}}_{actual}) \dot{\textbf{q}}_{actual} + G(\textbf{q}_{actual})
\end{align}
```

where M, C and G are the mass matrix, Coriolis matrix and gravity vector respectively. The torques are then sent to the robot.

## Usage

To use this controller, simply clone this repository in franka_ros and then make sure that the controller is loaded. For example,
you can add the following text to the default_controllers.yaml file in the `franka_ros/franka_control/config` directory:

```
panda_inverse_dynamics_controller:
  type: "panda_inverse_dynamics_controller/InverseDynamicsController"
  joint_names:
        - panda_joint1
        - panda_joint2
        - panda_joint3
        - panda_joint4
        - panda_joint5
        - panda_joint6
        - panda_joint7
  p_gains:
        - 300.0
        - 300.0
        - 300.0
        - 300.0
        - 150.0
        - 100.0
        - 750.0
  d_gains:
        - 40.0
        - 40.0
        - 40.0
        - 40.0
        - 20.0
        - 20.0
        - 10.0
```

The gain values are specified in the YAML file. These are the values I have found to work well so far.

## To Do
- [ ] Implement safety features for commanded velocity and make sure commanded positions are within joint limits

## Author
David Russell (scsdr@leeds.ac.uk)