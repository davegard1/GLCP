#include "integration.h"

// void integrate_explicit(System* system, StepperBase* stepper, const Eigen::VectorXd& x0, const double t0, const double dt0, const double tf, Eigen::VectorXd& xf, const bool print_on)
// {
// 	Eigen::VectorXd xold(x0.rows()), xnow(x0.rows()), xend(x0.rows()), xerr(x0.rows()), err_norm(x0.rows());
// 	double told, tnow, dt, scale, En;
// 	int dir;	
	
// 	if (tf - t0 > glob_ntol)
// 	{
// 		dir = 1;
// 	}
// 	else if(t0 - tf < -glob_ntol)
// 	{
// 		dir = -1;
// 	}	
// 	else //t0 & tf differ less than tolerance, xf = x0
// 	{
// 		xf = x0;
		
// 		return;
// 	}
	
	
// 	xold = x0;
// 	told = t0;
// 	dt = dt0;
	
// 	std::ofstream f_output;
// 	// Initiate Integration Options
// 	if (print_on)
// 	{
// 		std::string fname = "/home/davegard/projects/GCP/output.dat";
		
// 		f_output.open(fname);
// 	}

	
// 	if (dir == 1)
// 	{
// 		// forwards in double
		
// 		while(tf - tnow > glob_ntol)
// 		{
// 			// Print state
// 			if (f_output.is_open())
// 			{
// 				print_v2f(f_output, told, xold);
// 			}
			
			
// 			// check if dt > tf-tnow 
// 			if ( tnow + dt > tf )
// 			{
// 				dt = tf-tnow;
// 			}


// 			// Do step 
// 			stepper->do_step(system, told, xold, dt, xend, xerr);
			
// 			// Error Calcs
// 			err_norm << xerr.cwiseProduct(system->_dynamics_ptr->scales) ;

// 			En = err_norm.norm();
			
// 			// std::cout << stepper->order<<std::endl;
// 			scale = glob_SF*pow(glob_abs_tol/En,stepper->order);
			
// 			// Scale error & update states appropriately 
// 			// Could change to be more efficient
// 			if (scale >= glob_max_scale){
// 				// Good step, max scaling dt
				
// 				// update states
// 				tnow = told + dt;
// 				xnow = xend;
				
// 				// scale doublestep
// 				dt *= glob_max_scale;
// 			}
// 			else if (scale <= glob_min_scale){
// 				// bad step, min scaling dt
				
// 				// don't update states
// 				//tnow += dt;
// 				//xnow = xend;

//                 tnow = told;
//                 xnow = xold;
				
// 				// scale doublestep
// 				dt *= glob_min_scale;
// 			}
// 			else {
// 				//good step, within normal bounds
				
// 				// update states
// 				tnow = told + dt;
// 				xnow = xend;
				
// 				// scale doublestep
// 				dt *= scale;
				
// 			}
					
			
// 			// Update old states for next loop
// 			told = tnow;
// 			xold = xnow;
// 		}
		
		
// 		// Print out options here
// 		if (print_on)
// 		{
// 			if (f_output.is_open())
// 			{
// 				print_v2f(f_output, tnow, xnow);
// 			}
// 		}
		
// 	}
	
// 	else
// 	{
// 		// backwards in double 
// 		while(tnow - tf > glob_ntol)
// 		{
// 			// Print state
// 			if (f_output.is_open())
// 			{
// 				print_v2f(f_output, told, xold);
// 			}
			
// 			if ( tnow+dt < tf )
// 			{
// 				dt = tf-tnow;
// 			}
			
// 			stepper->do_step(system,tnow,xnow,dt,xend,xerr);
			
// 			err_norm << xerr.dot(system->_dynamics_ptr->scales);
			
// 			En = err_norm.norm();
			
// 			scale = glob_SF*pow(glob_abs_tol/En,0.125);
			
			
// 			// Scale error & update states appropriately 
// 			// Could change to be more efficient
// 			if (scale >= glob_max_scale){
// 				// Good step, max scaling dt
				
// 				// update states
// 				tnow = told + dt;
// 				xnow = xend;
				
// 				// scale doublestep
// 				dt *= glob_max_scale;
// 			}
// 			else if (scale <= glob_min_scale){
// 				// bad step, min scaling dt
				
// 				// don't update states
// 				//tnow += dt;
// 				//xnow = xend;
				
// 				// scale doublestep
// 				dt *= glob_min_scale;
// 			}
// 			else {
// 				//good step, within normal bounds
				
// 				// update states
// 				tnow = told + dt;
// 				xnow = xend;
				
// 				// scale doublestep
// 				dt *= scale;
				
// 			}

// 			// Update old states for next loop
// 			told = tnow;
// 			xold = xnow;			
// 		}
		
// 		// Print out options here
// 		if (print_on)
// 		{
// 			if (f_output.is_open())
// 			{
// 				print_v2f(f_output, tnow, xnow);
// 			}
// 		}
		
// 	}
	
// 	// Final output vars
// 	// tend = tnow;
// 	xf = xnow;

// 	if (print_on)
// 	{
// 		f_output.close();
// 	}
// }



