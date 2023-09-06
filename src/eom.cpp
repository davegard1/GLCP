#include "eom.h"


// void CartPole::rates( const double t , const Eigen::VectorXd &state, Eigen::VectorXd &rates)
void CartPole::operator()( const double t , const Eigen::VectorXd &state, Eigen::VectorXd &rates)
{
	// Input states
	double x1, x2, x3, x4, u1{0.};

	x1 = state[0]; // cart position
	x2 = state[1]; // cart speed
	x3 = state[2]; // pendulum angle
	x4 = state[3]; // pendulum angle rate of change

	// Intermediate states
	double denom;

	denom = m_ * L_ * L_ * (M_ + m_ * ( 1 - pow( cos(x3) , 2) ) );
	
	// Get Control Input
	// get_control_input(t, state, u1);

	// Output states
	double y1, y2, y3, y4;

	y1 = x2;

	y2 = (-m_*m_*L_*L_*g_*cos(x3)*sin(x3) + m_*L_*L_* (m_*L_*x4*x4*sin(x3) - damp_*x2) + m_*L_*L_* u1) / denom;

	y3 = x4;

	y4 = ( ( m_ + M_) * m_ * g_ * L_ * sin(x3) - m_ * L_ * cos(x3) * ( m_ * L_ * x4 * x4 * sin(x3) - damp_ * x2) + m_ * L_ * cos(x3) * u1 ) / denom;

	// Rates
	rates << y1, y2, y3, y4;

		
	
};


void CartPole::get_control_input(const double t , const Eigen::VectorXd& state, double& u)
{
	u = -0.0;

}

void CartPole::get_linearized_system(Eigen::MatrixXd& A,  Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D) 
{
	A << 0.,	1.,								0., 							0.,
		 0.,	-damp_ / M_,					b_ * m_ * g_ / M_, 				0.,
		 0.,	0., 							0., 							1.,
		 0.,	-b_ * damp_ / (M_ * L_),	-b_ * (m_ + M_) * g_ / (M_ * L_),	0.;

	B << 0.,
		 1/M_,
		 0.,
		 b_ * 1/ (M_ * L_);

	C << 1.,
		 1.,
		 1.,
		 1.;

	D << 0.;


	// return 0;

}

// void CartPole::rates( const double t , const Eigen::VectorXd &state, Eigen::VectorXd &rates)
void CartPole2::operator()( const double t , const Eigen::VectorXd &state, Eigen::VectorXd &rates)
{
	// Input states
	double x1, x2, x3, x4, u1{0.};

	x1 = state[0]; // cart position
	x2 = state[1]; // cart speed
	x3 = state[2]; // pendulum angle
	x4 = state[3]; // pendulum angle rate of change

	// Intermediate states
	double denom;

	denom = 1. - m_*l_ / (Mt_*L_) * pow(cos(x3), 2) ;
	
	// Get Control Input
	// get_control_input(t, state, u1);

	// Output states
	double y1, y2, y3, y4;

	y1 = x2;

	y2 = (-m_*l_/(Mt_*L_) * g_*sin(x3)*cos(x3) + m_*l_/(Mt_) * sin(x3)*x4*x4 + 1./Mt_*u1) / denom;

	y3 = x4;

	y4 = (g_/L_*sin(x3) - -m_*l_/(Mt_*L_) * sin(x3)*cos(x3)*x4*x4 - 1./(Mt_*L_)*cos(x4)*u1) / denom;

	// Rates
	rates << y1, y2, y3, y4;

		
	
};


void CartPole2::get_linearized_system(Eigen::MatrixXd& A,  Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D) 
{
	A << 0.,	1.,							  0., 	0.,
		 0.,	0.,		-alpha_ * m_*l_/(Mt_*L_), 	0.,
		 0.,	0.,							  0., 	1.,
		 0.,	0.,				  alpha_ * g_/L_,	0.;

	B << 0.,
		 alpha_/Mt_,
		 0.,
		 -alpha_/(Mt_*L_);

	C << 1.,
		 0.,
		 1.,
		 0.;

	D << 0.;


	// return 0;

}
