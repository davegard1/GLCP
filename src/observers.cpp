
#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "observers.h"






KalmanFilter::KalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& C, const Eigen::MatrixXd& Vd, const Eigen::MatrixXd& Vn  ) 
{ 

	// Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x, dim_x);
	Eigen::MatrixXd P;
	// Solve Riccati equation to get optimal gain matrix
	solveRiccatiArimotoPotter(A.transpose(), C.transpose(), Vd, Vn, P, Kf_);

	Kf_ = Kf_.transpose();


}

