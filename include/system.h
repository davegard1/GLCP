#pragma once

#include <math.h>
#include <vector>
#include <iostream>
#include <string>


// #include "steppers.h"
#include "integration.h"
#include "eom.h"
#include "structs.h"
#include "controllers.h"
#include "observers.h"

#include <Eigen/Dense>
#include <random>





class CartPoleSystem
{
	
	public:
	
	// Constructor
	CartPoleSystem(int id) : ID(id), genVd_(1234), genVn_(12345), dis_(-1.0,1.0) {}
	// CartPoleSystem(int id) : ID(id) {}

	
	// Attributes 
	int ID {0};
	bool linear_ {false};
	bool active_controller_ {false};
	bool active_observer_ {false};
	const int dim_x_ {4};
	const int dim_u_ {1};

	// Target final position of the system
	Eigen::VectorXd xtarg_ {};

	// Disturbance and noise covariances
	Eigen::MatrixXd Vd_ = Eigen::MatrixXd::Zero(4, 4);
	Eigen::MatrixXd Vn_ = Eigen::MatrixXd::Zero(1, 1);

	// Array for processing noise covariances
	Eigen::MatrixXd n_ = Eigen::MatrixXd::Zero(1, 4);

	// Random number generator
	std::mt19937 genVd_;
	std::mt19937 genVn_;
	std::uniform_real_distribution<> dis_;
	
	// Rates to get system dynamics
	// void rates(const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates);
	void operator()(const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates);
	
	
	// (Dynamics)
	void set_base_dynamics(DynamicsBase* dynamics);
	DynamicsBase *_dynamics_ptr;


	// Add LQR Controller
	LQR *LQR_ptr = nullptr;
	void add_lqr(const Eigen::MatrixXd& Q, Eigen::MatrixXd& R);

	void disable_controller();

	// Linearized system matricies and xref
	LinSys linsys_;

	void set_lin_sys(const Eigen::VectorXd& xref);


	// Use linearized dynamics?
	void set_linear(const bool linearize);

	// Add Target xf
	void set_xtarg(const Eigen::VectorXd& xtarg);


	// Observer Methods
	KalmanFilter *KF_ptr = nullptr;
	void add_kalman_filter(const Eigen::MatrixXd& Vd, const Eigen::MatrixXd& Vn);

	// protected:

	

	private:

	
	

	
	
};

