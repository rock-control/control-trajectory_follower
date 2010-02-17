/*
 * =====================================================================================
 *
 *       Filename:  SimpleIntegrator.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  10/19/09 14:01:38
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu (), ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#include "RK4Integrator.hpp"

#ifndef  SIMPLEINTEGRATOR_H__INC
#define  SIMPLEINTEGRATOR_H__INC

/*
 * =====================================================================================
 *        Class:  SimpleIntegrator
 *  Description:  
 * =====================================================================================
 */
class SimpleIntegrator : public RK4_SIM
{
	public:
		/* ====================  LIFECYCLE     ======================================= */
		SimpleIntegrator ();
		SimpleIntegrator (double _sample_time, double _initial_time = 0.0, double _init_val = 0.0);                             /* constructor */

		/* ====================  MUTATORS      ======================================= */
		void init(double _sample_time, double _initial_time = 0.0, double _init_val = 0.0);

		// Overload DERIV function of RK4_SIM for the integrator
		void DERIV(const double t, const double *x, 
				const double *u, double *xdot);

		double update(double derivVal);
	protected:

	private:
}; /* -----  end of class VelocityPositionEstimator  ----- */


#endif   /* ----- #ifndef SIMPLEINTEGRATOR_H__INC  ----- */
