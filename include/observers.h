
#ifndef GLCP_OBSERVERS
#define GLCP_OBSERVERS


#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "riccati_solver.h"



// class DynamicsBase
// {
// 	public:
	
// 	// Method all dynamics must override, evaluation of dynamics
// 	// virtual void rates( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) = 0;
// 	virtual void operator()( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) = 0;
	
// 	// Attribute for scaling error calc and thus adaptive time step
// 	Eigen::VectorXd scales;
	
// 	// Constructor (needed to assign scales)
// 	DynamicsBase(const Eigen::VectorXd scales_init) : scales( scales_init ) {}
// };


// class ControllerBase
// {
// 	public:

// 	ControllerBase() {}


// };


class KalmanFilter
{
	
	public:
	
	Eigen::MatrixXd Kf_;
	
	// Eigen::VectorXd scales;
	
	// Constructor Declaration
	KalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C, const Eigen::MatrixXd& Vd, const Eigen::MatrixXd& Vn  );

	
};


#endif //observers