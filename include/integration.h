#pragma once

#include <math.h>
#include <iostream>
#include <string>
#include <fstream>
#include <vector>

//#include "boost/array.hpp"

#include "math.h"
#include "util.h"
#include "glob.h"
#include "steppers.h"
#include "eom.h"
#include "structs.h"
#include <Eigen/Dense>

	





// Explicit integration without events
template <class System, class Stepper>
void integrate_adaptive_explicit(System system, Stepper stepper, const Eigen::VectorXd& x0, const double t0, const double dt0, const double tf, Eigen::VectorXd& xf, OutputObs& output)
{
	Eigen::VectorXd xold(x0.rows()), xnow(x0.rows()), xend(x0.rows()), xerr(x0.rows()), err_norm(x0.rows());
	double told {t0}, tnow {t0}, dt {dt0}, scale, En;
	int dir;	
	
	// std::cout << "Here \n";
	// std::cout << "ts: " << t0 << " "<<dt0 << " "<< tf << std::endl;


	if (tf - t0 > glob_ntol)
	{
		dir = 1;
	}
	else if(t0 - tf < -glob_ntol)
	{
		dir = -1;
	}	
	else //t0 & tf differ less than tolerance, xf = x0
	{
		xf = x0;
		
		return;
	}
	
	
	xold = x0;
	told = t0;
	// dt = dt0;
	
	// std::ofstream f_output;
	// Initiate Integration Options
	if (output.print)
	{
		// std::string fname = "/home/davegard/projects/GCP/output.dat";
		
		output.f_output.open(output.fname);
	}

	
	if (dir == 1)
	{
		// forwards in double
		
		while(tf - tnow > glob_ntol)
		{
			// Print state
			if (output.f_output.is_open())
			{
				// print_v2f(f_output, told, xold);
				output.f_output << told << ", " << xold.format(output.fmt) << std::endl;
			}
			
			
			// check if dt > tf-tnow 
			if ( tnow + dt > tf )
			{
				dt = tf-tnow;
			}


			// Do step 
			stepper.do_step(system, told, xold, dt, xend, xerr);
			
			// Error Calcs
			// Moving scales to inside the dynamics/class/operator function
			// err_norm << xerr.cwiseProduct(system.scales) ;

			En = xerr.norm();
			
			// std::cout << stepper->order<<std::endl;
			scale = glob_SF*pow(glob_abs_tol/En,stepper.order);
			
			// Scale error & update states appropriately 
			// Could change to be more efficient
			if (scale >= glob_max_scale){
				// Good step, max scaling dt
				
				// update states
				tnow = told + dt;
				xnow = xend;
				
				// scale doublestep
				dt *= glob_max_scale;
			}
			else if (scale <= glob_min_scale){
				// bad step, min scaling dt
				
				// don't update states
				//tnow += dt;
				//xnow = xend;

                tnow = told;
                xnow = xold;
				
				// scale doublestep
				dt *= glob_min_scale;
			}
			else {
				//good step, within normal bounds
				
				// update states
				tnow = told + dt;
				xnow = xend;
				
				// scale doublestep
				dt *= scale;
				
			}
					
			
			// Update old states for next loop
			told = tnow;
			xold = xnow;
		}
		
		
		// Print out options here
		if (output.print)
		{
			if (output.f_output.is_open())
			{
				// print_v2f(f_output, tnow, xnow);
				output.f_output << tnow << ", " << xnow.format(output.fmt) << std::endl;
			}
		}
		
	}
	
	else
	{
		// backwards in double 
		while(tnow - tf > glob_ntol)
		{
			// Print state
			if (output.f_output.is_open())
			{
				// print_v2f(f_output, told, xold);
				output.f_output << told << ", " << xold.format(output.fmt) << std::endl;

			}
			
			if ( tnow+dt < tf )
			{
				dt = tf-tnow;
			}
			
			stepper.do_step(system,tnow,xnow,dt,xend,xerr);
			
			// Moving scales to inside the dynamics/class/operator function
			// err_norm << xerr.cwiseProduct(system.scales);
			
			En = xerr.norm();
			
			scale = glob_SF*pow(glob_abs_tol/En, stepper.order);
			
			
			// Scale error & update states appropriately 
			// Could change to be more efficient
			if (scale >= glob_max_scale){
				// Good step, max scaling dt
				
				// update states
				tnow = told + dt;
				xnow = xend;
				
				// scale doublestep
				dt *= glob_max_scale;
			}
			else if (scale <= glob_min_scale){
				// bad step, min scaling dt
				
				// don't update states
				//tnow += dt;
				//xnow = xend;
				
				// scale doublestep
				dt *= glob_min_scale;
			}
			else {
				//good step, within normal bounds
				
				// update states
				tnow = told + dt;
				xnow = xend;
				
				// scale doublestep
				dt *= scale;
				
			}

			// Update old states for next loop
			told = tnow;
			xold = xnow;			
		}
		
		// Print out options here
		if (output.print)
		{
			if (output.f_output.is_open())
			{
				// print_v2f(f_output, tnow, xnow);
				output.f_output << tnow << ", " << xnow.format(output.fmt) << std::endl;
			}
		}
		
	}
	
	// Final output vars
	// tend = tnow;
	xf = xnow;

	if (output.print)
	{
		output.f_output.close();
	}
}


// Explicit integration without events
template <class System, class Stepper>
void integrate_fixed_explicit(System system, Stepper stepper, const Eigen::VectorXd& x0, const double t0, const double dt0, const double tf, Eigen::VectorXd& xf, OutputObs& output)
{
	Eigen::VectorXd xold(x0.rows()), xnow(x0.rows()), xend(x0.rows()), xerr(x0.rows()), err_norm(x0.rows());
	double told {t0}, tnow {t0}, dt {dt0}, scale, En;
	int dir;	
	
	// std::cout << "Here \n";
	// std::cout << "ts: " << t0 << " "<<dt0 << " "<< tf << std::endl;


	if (tf - t0 > glob_ntol)
	{
		dir = 1;
	}
	else if(t0 - tf < -glob_ntol)
	{
		dir = -1;
	}	
	else //t0 & tf differ less than tolerance, xf = x0
	{
		xf = x0;
		
		return;
	}
	
	
	xold = x0;
	told = t0;
	// dt = dt0;
	
	// std::ofstream f_output;
	// Initiate Integration Options
	if (output.print)
	{
		// std::string fname = "/home/davegard/projects/GCP/output.dat";
		
		output.f_output.open(output.fname);
	}

	
	if (dir == 1)
	{
		// forwards in double
		
		while(tf - tnow > glob_ntol)
		{
			// Print state
			if (output.f_output.is_open())
			{
				// print_v2f(f_output, told, xold);
				output.f_output << told << ", " << xold.format(output.fmt) << std::endl;

			}
			
			// check if dt > tf-tnow 
			if ( tnow + dt > tf )
			{
				dt = tf-tnow;
			}


			// Do step 
			stepper.do_step(system, told, xold, dt, xend, xerr);
			
			// update states
			tnow = told + dt;
			xnow = xend;
				
					
			
			// Update old states for next loop
			told = tnow;
			xold = xnow;
		}
		
		
		// Print out options here
		if (output.f_output.is_open())
		{
			// print_v2f(f_output, told, xold);
			output.f_output << tnow << ", " << xnow.format(output.fmt) << std::endl;

		}
		
	}
	
	else
	{
		// backwards in double 
		while(tnow - tf > glob_ntol)
		{
			// Print state
			if (output.f_output.is_open())
			{
				// print_v2f(f_output, told, xold);
				output.f_output << told << ", " << xold.format(output.fmt) << std::endl;

			}
			
			if ( tnow+dt < tf )
			{
				dt = tf-tnow;
			}
			
			stepper.do_step(system,tnow,xnow,dt,xend,xerr);
				
			// update states
			tnow = told + dt;
			xnow = xend;
				
			
			// Update old states for next loop
			told = tnow;
			xold = xnow;			
		}
		
		// Print out options here
		if (output.f_output.is_open())
		{
			// print_v2f(f_output, told, xold);
			output.f_output << tnow << ", " << xnow.format(output.fmt) << std::endl;

		}
		
	}
	
	// Final output vars
	// tend = tnow;
	xf = xnow;

	if (output.print)
	{
		output.f_output.close();
	}
}

