
#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "controllers.h"






LQR::LQR(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R  ) 
{ 

	// Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dim_x, dim_x);
	Eigen::MatrixXd P;
	// Solve Riccati equation to get optimal gain matrix
	solveRiccatiArimotoPotter(A, B, Q, R, P, m_K);


}

