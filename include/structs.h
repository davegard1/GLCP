#pragma once

#include <Eigen/Dense>

struct LinSys
{
	Eigen::VectorXd xref;
	Eigen::MatrixXd A;
	Eigen::MatrixXd B;
	Eigen::MatrixXd C;
	Eigen::MatrixXd D;
};


struct OutputObs
{
	bool print {true};
	std::ofstream f_output;
	std::string fname;
	Eigen::IOFormat fmt{Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", ""};
};

