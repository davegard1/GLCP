#include <iostream>
#include <iterator>
#include <array>
#include <vector>
#include <chrono> // for std::chrono functions
#include <cstddef> // for std::size_t
#include <numeric> // for std::iota
#include <algorithm>
#include <cmath>
#include <random>

// GLCP headers
#include "eom.h"
#include "util.h"
#include <Eigen/Dense>
#include "riccati_solver.h"
#include "system.h"
#include "integration.h"
#include "steppers.h"
#include "observers.h"



int main()
{
    // Initialize state, t0, and xdot
    Eigen::VectorXd x0(4);
    x0 <<  -1.0 , 0.0, 0., 0.;
    double t0 = 0.;
    Eigen::VectorXd xdot(4);
    xdot << 0. , 0., 0., 0.;

    // Initialize the underlying cartpole dynamics
    CartPole2 cartpole(1., 5., 1., 0.25, -9.81);


    // Define dimensions of carpole system
    const int dim_x = 4;
    const int dim_u = 1;


    // Define cost influence matricies for LQR
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x, dim_x);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_u, dim_u);

    Q(0, 0) = 1.0;
    Q(1, 1) = 1.0;
    Q(2, 2) = 1.0;
    Q(3, 3) = 1.0;

    R(0, 0) = 0.5;

    // Initialize Eigen output format
    Eigen::IOFormat CommaInitFmt{Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", ""};

    // Initialize the carpole system
    CartPoleSystem cartpole_system(1);

    cartpole_system.set_base_dynamics(&cartpole);
    cartpole_system.set_linear(false);
    

    // Set the system to be linearized about fixed point [0, 0, pi, 0]
    Eigen::VectorXd xref(4);
    xref <<  0. , 0.0, 0., 0.;
    cartpole_system.set_lin_sys(xref);

    // Add LQR to the system
    cartpole_system.add_lqr(Q, R);

    // Set target xf
    Eigen::VectorXd xtarg(4);
    xtarg <<  10. , 0.0, 0., 0.;

    cartpole_system.set_xtarg(xtarg);
    // cartpole_system.set_stepper_rk54();

    // Initialize the stepper
    RK54_stepper rk54;

    // Initialize xf
    Eigen::VectorXd xf(4);

    // Initialize output observer to print data
    OutputObs output;
    output.fname = "../Cartpole_LQR.dat";
    


    // Integrate the system from t0 = 0.0 to t0 = 10.0 with dt0 = 0.01
    integrate_fixed_explicit(cartpole_system, rk54, x0, 0.0 , .01,  10.0, xf, output);
    std::cout << "Xf: "<<  xf << std::endl;




    // System with noise
    Eigen::MatrixXd Vd = Eigen::MatrixXd::Identity(dim_x, dim_x) * 0.5;
    Eigen::MatrixXd Vn = Eigen::MatrixXd::Identity(dim_u, dim_u) * 0.5;

    // Initialize Kalman filter
    cartpole_system.add_kalman_filter(Vd, Vn);

    // Integrate with non-linear dynamics
    cartpole_system.set_linear(true);


    Eigen::VectorXd x02(8);
    x02 <<  -1.0 , 0.0, 0.0, 0., -1.0, 0.0, 0.1, 0.0;

    Eigen::VectorXd xf2(8);


    // Initialize output observer to print data
    OutputObs output2;
    output2.fname = "../Cartpole_LQG.dat";

    // Integrate the system from t0 = 0.0 to t0 = 10.0 with dt0 = 0.01
    integrate_fixed_explicit(cartpole_system, rk54, x02, 0.0 , 0.01, 10.0, xf2, output2);

    std::cout << "Xf: "<<  xf2.format(CommaInitFmt) << std::endl;




    return 0;
}