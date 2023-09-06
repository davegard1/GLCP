#ifndef STOP_INTEGRATION_STEPPERS
#define STOP_INTEGRATION_STEPPERS

#include <vector>

// #include "integration/rk87ve.h"



#include "eom.h"
#include "math.h"
#include "util.h"
#include <Eigen/Dense>
#include <functional>

#include "rk87ve.h"
#include "rk65ve.h"
#include "rk54.h"

// Removing Base stepper class so we can templatetize the funtion call (to get rates)
// Would need to template a virtual method which is currently not allowed
// Now steppers need to be vigilant to all have same functionality with no base class

// class StepperBase
// {
	
// 	public:
	
// 	// template <class System, class std::vector<double>, class double>
// 	// virtual void do_step(System system, double &t0, std::vector<double> &x0, double &dt, std::vector<double> &xend, std::vector<double> &xerr) = 0;
	
// 	const double order;
	
// 	StepperBase(const double order_init) : order(order_init) {}
	
// 	template <class Fun>
// 	virtual void do_step(Fun f, const double t0, const Eigen::VectorXd &x0, const double dt, Eigen::VectorXd& xend, Eigen::VectorXd& xerr) = 0;
	
	
	
	
// };



class RK87ve_stepper
{
	public:

	const double order;
	
	template <class Fun>
	void do_step(Fun f, double t0, const Eigen::VectorXd& x0, const double dt, Eigen::VectorXd& xend, Eigen::VectorXd& xerr);
	
	RK87ve_stepper() : order (1.0/8.0) {}
	
	
};

class RK65ve_stepper
{
	public:

	const double order;
	
	template <class Fun>
	void do_step(Fun f, double t0, const Eigen::VectorXd& x0, const double dt, Eigen::VectorXd& xend, Eigen::VectorXd& xerr);
	
	RK65ve_stepper() : order (1.0/6.0) {}
	
	
};

class RK54_stepper
{
	public:

	const double order;
	
	template <class Fun>
	void do_step(Fun f, double t0, const Eigen::VectorXd& x0, const double dt, Eigen::VectorXd& xend, Eigen::VectorXd& xerr);
	
	RK54_stepper() : order (1.0/4.0) {}
	
	
};
















template <class Fun>
void RK87ve_stepper::do_step(Fun f, double t0, const Eigen::VectorXd& x0, const double dt, Eigen::VectorXd& xend, Eigen::VectorXd& xerr)
{
	Eigen::VectorXd k1(x0.rows()), k2(x0.rows()), k3(x0.rows()), k4(x0.rows()), k5(x0.rows()), k6(x0.rows()), k7(x0.rows()), k8(x0.rows()), k9(x0.rows()), k10(x0.rows()), k11(x0.rows()), k12(x0.rows()), k13(x0.rows()), bl(x0.rows());
	
	
	f(t0, x0, k1);
	f(t0 + rk87ve_c[1] *dt , x0 + dt*(rk87ve_a2 [0]*k1), k2);
	f(t0 + rk87ve_c[2] *dt , x0 + dt*(rk87ve_a3 [0]*k1 + rk87ve_a3 [1]*k2), k3);
	f(t0 + rk87ve_c[3] *dt , x0 + dt*(rk87ve_a4 [0]*k1 + rk87ve_a4 [1]*k2 + rk87ve_a4 [2]*k3), k4);
	f(t0 + rk87ve_c[4] *dt , x0 + dt*(rk87ve_a5 [0]*k1 + rk87ve_a5 [1]*k2 + rk87ve_a5 [2]*k3 + rk87ve_a5 [3]*k4), k5);
	f(t0 + rk87ve_c[5] *dt , x0 + dt*(rk87ve_a6 [0]*k1 + rk87ve_a6 [1]*k2 + rk87ve_a6 [2]*k3 + rk87ve_a6 [3]*k4 + rk87ve_a6 [4]*k5), k6);
	f(t0 + rk87ve_c[6] *dt , x0 + dt*(rk87ve_a7 [0]*k1 + rk87ve_a7 [1]*k2 + rk87ve_a7 [2]*k3 + rk87ve_a7 [3]*k4 + rk87ve_a7 [4]*k5 + rk87ve_a7 [5]*k6), k7);
	f(t0 + rk87ve_c[7] *dt , x0 + dt*(rk87ve_a8 [0]*k1 + rk87ve_a8 [1]*k2 + rk87ve_a8 [2]*k3 + rk87ve_a8 [3]*k4 + rk87ve_a8 [4]*k5 + rk87ve_a8 [5]*k6 + rk87ve_a8 [6]*k7), k8);
	f(t0 + rk87ve_c[8] *dt , x0 + dt*(rk87ve_a9 [0]*k1 + rk87ve_a9 [1]*k2 + rk87ve_a9 [2]*k3 + rk87ve_a9 [3]*k4 + rk87ve_a9 [4]*k5 + rk87ve_a9 [5]*k6 + rk87ve_a9 [6]*k7 + rk87ve_a9 [7]*k8), k9);
	f(t0 + rk87ve_c[9] *dt , x0 + dt*(rk87ve_a10[0]*k1 + rk87ve_a10[1]*k2 + rk87ve_a10[2]*k3 + rk87ve_a10[3]*k4 + rk87ve_a10[4]*k5 + rk87ve_a10[5]*k6 + rk87ve_a10[6]*k7 + rk87ve_a10[7]*k8 + rk87ve_a10[8]*k9), k10);
	f(t0 + rk87ve_c[10]*dt , x0 + dt*(rk87ve_a11[0]*k1 + rk87ve_a11[1]*k2 + rk87ve_a11[2]*k3 + rk87ve_a11[3]*k4 + rk87ve_a11[4]*k5 + rk87ve_a11[5]*k6 + rk87ve_a11[6]*k7 + rk87ve_a11[7]*k8 + rk87ve_a11[8]*k9 + rk87ve_a11[9]*k10), k11);
	f(t0 + rk87ve_c[11]*dt , x0 + dt*(rk87ve_a12[0]*k1 + rk87ve_a12[1]*k2 + rk87ve_a12[2]*k3 + rk87ve_a12[3]*k4 + rk87ve_a12[4]*k5 + rk87ve_a12[5]*k6 + rk87ve_a12[6]*k7 + rk87ve_a12[7]*k8 + rk87ve_a12[8]*k9 + rk87ve_a12[9]*k10 + rk87ve_a12[10]*k11), k12);
	f(t0 + rk87ve_c[12]*dt , x0 + dt*(rk87ve_a13[0]*k1 + rk87ve_a13[1]*k2 + rk87ve_a13[2]*k3 + rk87ve_a13[3]*k4 + rk87ve_a13[4]*k5 + rk87ve_a13[5]*k6 + rk87ve_a13[6]*k7 + rk87ve_a13[7]*k8 + rk87ve_a13[8]*k9 + rk87ve_a13[9]*k10 + rk87ve_a13[10]*k11 + rk87ve_a13[11]*k12), k13);

	xend = x0 + dt*(rk87ve_bu[0]*k1 + rk87ve_bu[1]*k2 + rk87ve_bu[2]*k3 + rk87ve_bu[3]*k4 + rk87ve_bu[4]*k5 + rk87ve_bu[5]*k6 + rk87ve_bu[6]*k7 + rk87ve_bu[7]*k8 + rk87ve_bu[8]*k9 + rk87ve_bu[9]*k10 + rk87ve_bu[10]*k11 + rk87ve_bu[11]*k12 + rk87ve_bu[12]*k13);
	bl = x0 + dt*(rk87ve_bl[0]*k1 + rk87ve_bl[1]*k2 + rk87ve_bl[2]*k3 + rk87ve_bl[3]*k4 + rk87ve_bl[4]*k5 + rk87ve_bl[5]*k6 + rk87ve_bl[6]*k7 + rk87ve_bl[7]*k8 + rk87ve_bl[8]*k9 + rk87ve_bl[9]*k10 + rk87ve_bl[10]*k11 + rk87ve_bl[11]*k12 + rk87ve_bl[12]*k13);
	
	xerr = xend-bl;
	
};

template <class Fun>
void RK65ve_stepper::do_step(Fun f, double t0, const Eigen::VectorXd& x0, const double dt, Eigen::VectorXd& xend, Eigen::VectorXd& xerr)
{
	Eigen::VectorXd k1(x0.rows()), k2(x0.rows()), k3(x0.rows()), k4(x0.rows()), k5(x0.rows()), k6(x0.rows()), k7(x0.rows()), k8(x0.rows()), k9(x0.rows()), bl(x0.rows());
	
	
	f(t0, x0, k1);
	f(t0 + rk65ve_c[1] *dt , x0 + dt*(rk65ve_a2 [0]*k1), k2);
	f(t0 + rk65ve_c[2] *dt , x0 + dt*(rk65ve_a3 [0]*k1 + rk65ve_a3 [1]*k2), k3);
	f(t0 + rk65ve_c[3] *dt , x0 + dt*(rk65ve_a4 [0]*k1 + rk65ve_a4 [1]*k2 + rk65ve_a4 [2]*k3), k4);
	f(t0 + rk65ve_c[4] *dt , x0 + dt*(rk65ve_a5 [0]*k1 + rk65ve_a5 [1]*k2 + rk65ve_a5 [2]*k3 + rk65ve_a5 [3]*k4), k5);
	f(t0 + rk65ve_c[5] *dt , x0 + dt*(rk65ve_a6 [0]*k1 + rk65ve_a6 [1]*k2 + rk65ve_a6 [2]*k3 + rk65ve_a6 [3]*k4 + rk65ve_a6 [4]*k5), k6);
	f(t0 + rk65ve_c[6] *dt , x0 + dt*(rk65ve_a7 [0]*k1 + rk65ve_a7 [1]*k2 + rk65ve_a7 [2]*k3 + rk65ve_a7 [3]*k4 + rk65ve_a7 [4]*k5 + rk65ve_a7 [5]*k6), k7);
	f(t0 + rk65ve_c[7] *dt , x0 + dt*(rk65ve_a8 [0]*k1 + rk65ve_a8 [1]*k2 + rk65ve_a8 [2]*k3 + rk65ve_a8 [3]*k4 + rk65ve_a8 [4]*k5 + rk65ve_a8 [5]*k6 + rk65ve_a8 [6]*k7), k8);
	f(t0 + rk65ve_c[8] *dt , x0 + dt*(rk65ve_a9 [0]*k1 + rk65ve_a9 [1]*k2 + rk65ve_a9 [2]*k3 + rk65ve_a9 [3]*k4 + rk65ve_a9 [4]*k5 + rk65ve_a9 [5]*k6 + rk65ve_a9 [6]*k7 + rk65ve_a9 [7]*k8), k9);

	xend = x0 + dt*(rk65ve_bu[0]*k1 + rk65ve_bu[1]*k2 + rk65ve_bu[2]*k3 + rk65ve_bu[3]*k4 + rk65ve_bu[4]*k5 + rk65ve_bu[5]*k6 + rk65ve_bu[6]*k7 + rk65ve_bu[7]*k8 + rk65ve_bu[8]*k9 );
	bl = x0 + dt*(rk65ve_bl[0]*k1 + rk65ve_bl[1]*k2 + rk65ve_bl[2]*k3 + rk65ve_bl[3]*k4 + rk65ve_bl[4]*k5 + rk65ve_bl[5]*k6 + rk65ve_bl[6]*k7 + rk65ve_bl[7]*k8 + rk65ve_bl[8]*k9 );
	
	xerr = xend-bl;
	
};

template <class Fun>
void RK54_stepper::do_step(Fun f, double t0, const Eigen::VectorXd& x0, const double dt, Eigen::VectorXd& xend, Eigen::VectorXd& xerr)
{
	Eigen::VectorXd k1(x0.rows()), k2(x0.rows()), k3(x0.rows()), k4(x0.rows()), k5(x0.rows()), k6(x0.rows()), bl(x0.rows());

	// std::cout << x0<< std::endl;
	
	
	f(t0, x0, k1);
	f(t0 + rk54_c[1] *dt , x0 + dt*(rk54_a2 [0]*k1), k2);
	f(t0 + rk54_c[2] *dt , x0 + dt*(rk54_a3 [0]*k1 + rk54_a3 [1]*k2), k3);
	f(t0 + rk54_c[3] *dt , x0 + dt*(rk54_a4 [0]*k1 + rk54_a4 [1]*k2 + rk54_a4 [2]*k3), k4);
	f(t0 + rk54_c[4] *dt , x0 + dt*(rk54_a5 [0]*k1 + rk54_a5 [1]*k2 + rk54_a5 [2]*k3 + rk54_a5 [3]*k4), k5);
	f(t0 + rk54_c[5] *dt , x0 + dt*(rk54_a6 [0]*k1 + rk54_a6 [1]*k2 + rk54_a6 [2]*k3 + rk54_a6 [3]*k4 + rk54_a6 [4]*k5), k6);

	xend = x0 + dt*(rk54_bu[0]*k1 + rk54_bu[1]*k2 + rk54_bu[2]*k3 + rk54_bu[3]*k4 + rk54_bu[4]*k5 + rk54_bu[5]*k6);
	bl = x0 + dt*(rk54_bl[0]*k1 + rk54_bl[1]*k2 + rk54_bl[2]*k3 + rk54_bl[3]*k4 + rk54_bl[4]*k5 + rk54_bl[5]*k6);
	
	xerr = xend-bl;
	
};



#endif //STOP_INTEGRATION_STEPPERS




