# panda_inverse_dynamics_controller
An inverse dynamics ROS controller for the Franka-Emika Panda. To be used in franka_ros.

```math
\begin{align}
\min_{u_t} & \sum_{t=0}^{T} \ell(x_t, u_t) + \phi(x_{T+1}), \\
\mathrm{s.t.}~& x_{t+1} = f(x_t, u_t),
\end{align}
```
## Usage

## Recommended Gains

##

## To Do
- [ ] Implement safety features for commanded velocity and make sure commanded positions are within joint limits
- [ ] Add equations to the ReadMe

## Author
David Russell (scsdr@leeds.ac.uk)