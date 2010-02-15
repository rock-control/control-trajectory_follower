/*
 * =====================================================================================
 *
 *       Filename:  SimpleIntegrator.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/19/09 14:01:31
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu (), ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */
#include "simpleintegrator.h"

SimpleIntegrator::SimpleIntegrator() : RK4_SIM(1, 1)
{
}  /* -----  end of method VelocityIntegrator::VelocityIntegrator  (constructor)  ----- */

SimpleIntegrator::SimpleIntegrator(double _sample_time, double _initial_time, double _init_val) : RK4_SIM(1, 1)
{
	init(_sample_time,_initial_time,_init_val);
}  /* -----  end of method VelocityIntegrator::VelocityIntegrator  (constructor)  ----- */

void SimpleIntegrator::init(double _sample_time, double _initial_time, double _init_val)
{
	RK4_SIM::init_param(_sample_time, _initial_time, &_init_val);
}

void SimpleIntegrator::DERIV(const double t, const double *x, 
		const double *u, double *xdot)
{
	// Implement integrator G(s) = 1/s
	xdot[0] = u[0];    
}

double SimpleIntegrator::update(double derivVal)
{
  // Find the integral of the input
  ctrl_input[0] = derivVal;
  solve();
  return plant_state[0];
}	
