
#ifndef GLCP_EOM
#define GLCP_EOM


#include <math.h>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>




class DynamicsBase
{
	public:
	
	// Method all dynamics must override, evaluation of dynamics
	// virtual void rates( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) = 0;
	virtual void operator()( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) = 0;

	// Linearization of Dynamics to get A & B plant matricies
	virtual void get_linearized_system(Eigen::MatrixXd& A,  Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D) = 0;
	
	// Attribute for scaling error calc and thus adaptive time step
	Eigen::VectorXd scales;
	
	// Constructor (needed to assign scales)
	DynamicsBase(const Eigen::VectorXd scales_init) : scales( scales_init ) {}
};




class CartPole: public DynamicsBase
{
	
	public:
	
	const double m_;
	const double M_;
	const double L_;
	const double g_;
	const double damp_;
	double b_;
	
	// Eigen::VectorXd scales;
	
	CartPole(const double m, const double M, const double L, const double g, const double damp, const double b ) : 
		m_ {m}, M_ {M}, L_ {L}, g_ {g}, damp_ {damp}, b_ {b}, 
		DynamicsBase( (Eigen::VectorXd(4) << 1.0e0, 1.0e0, 1.0e0, 1.0e0).finished() ) { }

    void operator()( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) override;
	
	// void rates( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) override;
	
	void get_control_input(const double t , const Eigen::VectorXd& state, double& u);

	void get_linearized_system(Eigen::MatrixXd& A,  Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D) override;

	
};

class CartPole2: public DynamicsBase
{
	
	public:
	
	const double m_;
	const double Mt_;
	const double l_;
	const double L_;
	const double g_;
	const double alpha_;
	
	// Eigen::VectorXd scales;
	
	CartPole2(const double m, const double M, const double l, const double I, const double g ) : 
		m_ {m},
		l_ {l}, 
		g_ {g},
		Mt_ { M+m },
		L_ { (I+m_*l_*l_) / (m_*l_) },
		alpha_ {1. / (1. - m_*l_/(Mt_*L_) )},  
		DynamicsBase( (Eigen::VectorXd(4) << 1.0e0, 1.0e0, 1.0e0, 1.0e0).finished() ) {}
	// CartPole2(const double m, const double M, const double l, const double I, const double g );

    void operator()( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) override;
	
	// void rates( const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates) override;
	
	void get_control_input(const double t , const Eigen::VectorXd& state, double& u);

	void get_linearized_system(Eigen::MatrixXd& A,  Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D) override;

	
};


#endif //GLCP_EOM