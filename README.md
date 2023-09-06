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
!(/cartpole_system.png)

The demo executable shows how to define the weighting Q & R matricies for your LQR controller and how to specify disturbance and noise covariances.

Results of the non-noisy system
!(/lqr_res.png)

Results of the noisy system
!(/lqg_res.png)
