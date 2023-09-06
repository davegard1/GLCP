# GLCP
Simple repo demoing LGR and LQG for a cartpole system

Repo shows basic linear control with a LQR, Kalman Filter, and LQG implementation for a cartpole system.

Noteable features include:
- Riccati equation solver using eigen value decomposition method
- Fixed and adaptive step explicit integration with RK54, RK65 (Verner), and RK87 (Verner) methods
- Use of Eigen for matricies and linear algebra
- Cartpole system class, ability to add/remove different LQR controllers or Kalman observers

The cartpole_demo.cpp shows a basic linear trajectory problem: 

Move the cart from [-1., 0., 0., 0.] to [10., 0.0 , 0.0, 0.0]
<p align="center">
    <img src="cartpole_system.png" height="150">
</p>


The demo executable shows how to define the weighting Q & R matricies for your LQR controller and how to specify disturbance and noise covariances.

Results of the non-noisy system
<p align="center">
    <img src="lqr_res.png" height="150">
</p>

Results of the noisy system
<p align="center">
    <img src="lqg_res.png" height="150">
</p>
