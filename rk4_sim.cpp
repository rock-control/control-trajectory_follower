/*
 * FILE --- rk4_sim.cc
 *
 * PURPOSE --- Implements 4th order Runge-Kutta differential equation
 * solver of an n'th order system.  User writes virtual function DERIV() 
 * containing system dynamics in the form of n first order differential  
 * equations, or x' = f(t,x,u). Therefore, if the initial equation is in 
 * the form of n'th order differential equation, it must be converted to 
 * a system of n first order differential equations. 
 *
 * Example : 
 * Given the differential equation
 * W'' + aW' + bW = cF 
 * Define
 * X1 = W 
 * X2 = W' 
 * To obtain the system of equations 
 * X1' = X2 
 * X2' = -aX2 - bX1 + cF 
 *
 * If your system is laready in the form x' = f(t,x,u) you dont have 
 * to do any transformation.
 */

// Include Files
#include "rk4_sim.h"


// Define this macro if you want to use this utility without linking
// to the RCS Library

#define NO_RCS

#ifdef NO_RCS
#include <stdio.h>            /* printf function */
#else
#include "rcs_prnt.hh"          /* rcs_print_ functions */
#endif


/*************************************************************/

// Constructor
// The arguments are self-explanatory
// If you specify initial conditions, the parameter _initial_state 
// should point to a vector of size _plant_order, so that correct 
// initialization could be performed.
RK4_SIM::RK4_SIM(int _plant_order, 
		int _ctrl_order,
		double _integration_step, 
		double _initial_time, 
		double *_initial_state)  
{
	// No error at start
	rk4_sim_err = RK4_SIM_NO_ERROR;

	// Initialize plant parameters
	plant_order = _plant_order;
	ctrl_order = _ctrl_order;

	// Allocate memory 
	allocate_memory();

	// If error then report
	if (rk4_sim_err != RK4_SIM_NO_ERROR) 
	{
#ifdef NO_RCS
		printf("RK4_SIM: ERROR --- Not Enought Memory!!!\n");
		printf("The Runge-Kutta Simulator will not function correctly ...\n");
#else
		rcs_print("RK4_SIM: ERROR --- Not Enought Memory!!!\n");
		rcs_print("The Runge-Kutta Simulator will not function correctly ...\n");
#endif
	}

	// Set the parameters and inital conditions
	init_param(_integration_step, _initial_time, _initial_state);
}

RK4_SIM::~RK4_SIM()
{
	// Free the allocated memory
	free_memory();
}

void RK4_SIM::allocate_memory(void)
{
	if ((plant_state = new double[plant_order]) == NULL)
		rk4_sim_err = RK4_SIM_MEM_ERROR;

	if ((f1 = new double[plant_order]) == NULL)
		rk4_sim_err = RK4_SIM_MEM_ERROR;

	if ((f2 = new double[plant_order]) == NULL)
		rk4_sim_err = RK4_SIM_MEM_ERROR;

	if ((f3 = new double[plant_order]) == NULL)
		rk4_sim_err = RK4_SIM_MEM_ERROR;

	if ((f4 = new double[plant_order]) == NULL)
		rk4_sim_err = RK4_SIM_MEM_ERROR;

	if ((temp = new double[plant_order]) == NULL)
		rk4_sim_err = RK4_SIM_MEM_ERROR;

	if ((ctrl_input = new double[ctrl_order]) == NULL)
		rk4_sim_err = RK4_SIM_MEM_ERROR;
}

void RK4_SIM::free_memory(void)
{
	delete [] plant_state;
	delete [] f1;
	delete [] f2;
	delete [] f3;
	delete [] f4;
	delete [] temp;
	delete [] ctrl_input;
}

void RK4_SIM::init_param(double _integration_step, 
		double _initial_time, 
		double *_initial_state)
{
	int ii;   // Index variable

	integration_step = _integration_step;
	current_time = _initial_time; 

	if (_initial_state == NULL) // No initial conditions specified
	{
		for (ii=0; ii < plant_order; ii++)
		{
			plant_state[ii] = 0.0;
		}
	}
	else // Initialize with the provided initial conditions
	{
		for (ii=0; ii < plant_order; ii++)
		{
			plant_state[ii] = _initial_state[ii];
		}
	}

	// Initial control is zero
	for (ii=0; ii < ctrl_order; ii++)
	{
		ctrl_input[ii] = 0.0;
	}
}

void RK4_SIM::solve (void)
{
	int ii;   // Index variable

	// Determine Runge-Kutta coefficients
	F1 ();
	F2 ();
	F3 ();
	F4 ();

	// Update time and output values
	current_time += integration_step;
	for (ii=0; ii < plant_order; ii++)
	{
		plant_state[ii] = plant_state[ii] + 
			(1.0/6.0) * (f1[ii] + 2.0*f2[ii] + 2.0*f3[ii] + f4[ii]);
	}
}

inline void RK4_SIM::F1 (void)
{
	int ii;   // Index variable

	DERIV (current_time, plant_state, ctrl_input, f1);
	for (ii=0; ii < plant_order; ii++)
	{
		f1[ii] = integration_step * f1[ii];
	}
}

inline void RK4_SIM::F2 (void)
{
	int ii;   // Index variable

	for (ii=0; ii < plant_order; ii++)
	{
		temp[ii] = plant_state[ii] + 0.5*f1[ii];
	}

	DERIV ((current_time + 0.5*integration_step), temp, ctrl_input, f2);

	for (ii=0; ii < plant_order; ii++)
	{
		f2[ii] = integration_step * f2[ii];
	}
}

inline void RK4_SIM::F3 (void)
{
	int ii;   // Index variable

	for (ii=0; ii < plant_order; ii++)
	{
		temp[ii] = plant_state[ii] + 0.5*f2[ii];
	}

	DERIV ((current_time + 0.5*integration_step), temp, ctrl_input, f3);

	for (ii=0; ii < plant_order; ii++)
	{
		f3[ii] = integration_step * f3[ii];
	}
}

inline void RK4_SIM::F4 (void)
{
	int ii;   // Index variable

	for (ii=0; ii < plant_order; ii++)
	{
		temp[ii] = plant_state[ii] + f3[ii];
	}

	DERIV ((current_time + integration_step), temp, ctrl_input, f4);

	for (ii=0; ii < plant_order; ii++)
	{
		f4[ii] = integration_step * f4[ii];
	}
}











