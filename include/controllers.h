
#ifndef controllers
#define controllers


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


class LQR
{
	
	public:
	
	Eigen::MatrixXd m_K;
	
	// Eigen::VectorXd scales;
	
	// Constructor Declaration
	LQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R  );

	
};

// class PID
// {
	
// 	public:
	
// 	double Kp_;
// 	double Ki_;
// 	double Kd_;

// 	Eigen::VectorXd xtarg_;


	
// 	// Eigen::VectorXd scales;
	
// 	// Constructor Declaration
// 	LQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R  );

	
// };


#endif //controllers