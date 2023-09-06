#include "system.h"


using namespace std;


// (INTEGRATION METHODS)
// void System::integrate(Eigen::VectorXd& state_init, const double t0, const double tf, Eigen::VectorXd& state_end, const bool print_on)
// {
	
// 	// Initialize dt to 100s, then convert to system/dynamics units with DU,TU
// 	double dt0 = (tf-t0) / 1e3;

// 	integrate_explicit(this->rates, _stepper_ptr, state_init, t0, dt0, tf, state_end, print_on);

// }


// void System::set_stepper_rk87ve()
// {
// 	_stepper_ptr = new RK87ve_stepper();
// }

// void System::set_stepper_rk65ve()
// {
// 	_stepper_ptr = new RK65ve_stepper();
// }

// void System::set_stepper_rk54()
// {
// 	_stepper_ptr = new RK54_stepper();
// }

// CartPoleSystem::CartPoleSystem(int id) : ID(id)
// {

// 	Vd_ = Eigen::MatrixXd::Zero(4, 4);
// 	Vn_ = Eigen::MatrixXd::Zero(1, 1);

// }



// (DYNAMICS METHODS)
void CartPoleSystem::set_base_dynamics(DynamicsBase* dynamics)
{
	_dynamics_ptr = dynamics;
}

// void System::rates(const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates)
void CartPoleSystem::operator()(const double t , const Eigen::VectorXd& state, Eigen::VectorXd& rates)
{
	if (!(linear_))
	{
		if (!(active_observer_))
		{
			// Non-linear dynamics with no disturbances/observer (just integrate xdot = f(x,u) ) 
			// std::cout << "Nonlinear \n";

			_dynamics_ptr->operator()(t, state, rates);

			// std::cout << "Rates pre control: "<< rates << "\n";


			if (active_controller_)
			{
				// std::cout << "u: " << - LQR_ptr->m_K * (state - xtarg_) << std::endl;
				rates += linsys_.B * (- LQR_ptr->m_K * (state - xtarg_) );
			}

			
			// std::cout << "Rates pre control: "<< (- LQR_ptr->m_K * (state - xtarg_) )<< "\n";
		}
		else
		{
			// Non-linear dynamics with disturbances/observer (xdot = f(x,u) -> y = C*xdot -> u = Klqr * y )
			// Rates [0:3] are the true states, our controller and observer can't see these
			// Rates [4:7] are the estimated states, what out controller and observer see/produce


			// _dynamics_ptr->operator()(t, state, rates);

			Eigen::MatrixXd y(1,4);
			Eigen::VectorXd rates_temp(4);
			
			// Output(s) of system that we can measure
			y = linsys_.C * state( Eigen::seq(0,3) ) /* + D * u */ + (Vn_ * Vn_ * dis_(genVn_));

			// LQR control based on estimated state - desired
			// u = LQR_ptr->m_K * (state(seq(4,7)) - xtarg_);

			// Rates of xhat
			// rates( Eigen::seq(4,7) ) = (linsys_.A - KF_ptr->Kf_ * linsys_.C - linsys_.B * LQR_ptr->m_K) * state( Eigen::seq(4,7) ) + KF_ptr->Kf_ * y;

			_dynamics_ptr->operator()(t, state(Eigen::seq(4,7)), rates_temp );

			rates( Eigen::seq(4,7) ) = rates_temp - ( KF_ptr->Kf_ * linsys_.C ) * state( Eigen::seq(4,7) ) + linsys_.B * (- LQR_ptr->m_K * (state( Eigen::seq(4,7) ) - xtarg_) ) + KF_ptr->Kf_ * y;


			// Generate disturbances on x
			Eigen::VectorXd xdis {{ dis_(genVn_), dis_(genVn_), dis_(genVn_), dis_(genVn_) }};

			// Rates of x (true x). Controller based off of estimated states 
			_dynamics_ptr->operator()(t, state(Eigen::seq(0,3)), rates_temp );
			rates( Eigen::seq(0,3) ) = rates_temp + linsys_.B * (- LQR_ptr->m_K * (state( Eigen::seq(4,7) ) - xtarg_) ) + (Vd_ * Vd_ * xdis);  


		}


	}
	else
	{
		// Linear dynamics
		if (!(active_observer_))
		{
			rates = linsys_.A * state;

			if (active_controller_)
			{
				rates += linsys_.B * (- LQR_ptr->m_K * (state - xtarg_) );
			}
		}
		else 
		{
			// Linear dynamics with disturbances/observer (xdot = A*x + B*u -> y = C*xdot -> u = Klqr * y )
			// Rates [0:3] are the true states, our controller and observer can't see these
			// Rates [4:7] are the estimated states, what out controller and observer see/produce


			// _dynamics_ptr->operator()(t, state, rates);

			Eigen::MatrixXd y(1,4);

			// Output(s) of system that we can measure
			y = linsys_.C * state( Eigen::seq(0,3) ) /* + D * u */ + (Vn_ * Vn_ * dis_(genVn_));

			// LQR control based on estimated state - desired
			// u = LQR_ptr->m_K * (state(seq(4,7)) - xtarg_);

			// Rates of xhat
			// rates( Eigen::seq(4,7) ) = (linsys_.A - KF_ptr->Kf_ * linsys_.C - linsys_.B * LQR_ptr->m_K) * state( Eigen::seq(4,7) ) + KF_ptr->Kf_ * y;
			rates( Eigen::seq(4,7) ) = linsys_.A * state(Eigen::seq(4,7)) - ( KF_ptr->Kf_ * linsys_.C ) * state( Eigen::seq(4,7) ) + linsys_.B * (- LQR_ptr->m_K * (state( Eigen::seq(4,7) ) - xtarg_) ) + KF_ptr->Kf_ * y;

			// Generate disturbances on x
			Eigen::VectorXd xdis {{ dis_(genVn_), dis_(genVn_), dis_(genVn_), dis_(genVn_) }};

			// Rates of x (true x). Controller based off of estimated states 
			rates( Eigen::seq(0,3) ) = linsys_.A * state(Eigen::seq(0,3)) + linsys_.B * (- LQR_ptr->m_K * (state( Eigen::seq(4,7) ) - xtarg_) ) + (Vd_ * Vd_ * xdis);  

		}
	}
}


void CartPoleSystem::add_lqr(const Eigen::MatrixXd& Q, Eigen::MatrixXd& R)
{

	// Eigen::MatrixXd A(4,4), B(4,1), C(1,4), D(1,1);

	// _dynamics_ptr->get_linearized_system(A, B, C, D);

	LQR_ptr = new LQR(linsys_.A, linsys_.B, Q, R);

	active_controller_ = true;


}

void CartPoleSystem::set_linear(const bool linearize)
{
	linear_ = linearize;
}

void CartPoleSystem::disable_controller()
{
	active_controller_ = false;

	LQR_ptr = nullptr;
}

void CartPoleSystem::set_xtarg(const Eigen::VectorXd& xtarg)
{
	xtarg_ = xtarg;
}

void CartPoleSystem::set_lin_sys(const Eigen::VectorXd& xref)
{
	Eigen::MatrixXd A(dim_x_,dim_x_), B(dim_x_,dim_u_), C(1,dim_x_), D(1,dim_u_);

	_dynamics_ptr->get_linearized_system(A, B, C, D);

	linsys_.xref = xref;
	linsys_.A = A;
	linsys_.B = B;
	linsys_.C = C;
	linsys_.D = D;

}


void CartPoleSystem::add_kalman_filter(const Eigen::MatrixXd& Vd, const Eigen::MatrixXd& Vn)
{

	// Eigen::MatrixXd A(4,4), B(4,1), C(1,4), D(1,1);

	// _dynamics_ptr->get_linearized_system(A, B, C, D);

	KF_ptr = new KalmanFilter(linsys_.A, linsys_.C, Vd, Vn);

	active_observer_ = true;

	Vd_ = Vd;
	Vn_ = Vn;

	for ( int i{0}; i < linsys_.C.cols(); ++i )
	{
		if (abs(linsys_.C(i)) > 1e-16)
		{
			n_(i) = 1.;
		}  
		// ++i;
	}

}
