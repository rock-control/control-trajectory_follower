/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryController_P.cpp
 *
 *    Description:  Implementation of trajectory controller with orientation control
 *    				from 'Springer handbook of robotics' chapter 34 pg 805
 *
 *        Version:  1.0
 *        Created:  10/13/09 10:30:32
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu, ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#include "TrajectoryController_P.h"

using namespace TrajectoryController;

TrajectoryController_P::TrajectoryController_P ()
{
}  /* -----  end of method TrajectoryController_P::TrajectoryController_P  (constructor)  ----- */

	void
TrajectoryController_P::setConstants(double K2_val, double K3_val, double R_val, double r_val, double ul, double ll)
{
	K2 = K2_val;
	K3 = K3_val;

	R = R_val;
	r = r_val;
	
	u_limit = ul; // radians per second
	l_limit = ll; // radians per second 
} 

        Eigen::Vector2d	
TrajectoryController_P::update (double u1, double d, double theta_e, double c, double c_s )
{
 	double d_dot, s_dot, z2, z3, v1, v2, u2;

 	d_dot = u1 * sin(theta_e);
	s_dot = u1 * cos(theta_e) / (1.0-d*c);

	z2 = d;
	z3 = (1.0-(d*c))*tan(theta_e);

	v1 = u1 * cos(theta_e) /(1.0 - d*c);
	v2 = (-v1 * K2 * z2) - (fabs(v1) * K3 * z3);

	u2 = ((v2 + ((d_dot*c + d*c_s*s_dot)*tan(theta_e))) / ((1.0-d*c)*(1+pow(tan(theta_e),2)))) - (s_dot*c);

	vel_right = limit((u1 + R*u2)/r);
	vel_left  = limit((u1 - R*u2)/r);

	return Eigen::Vector2d(u1, u2);
}		/* -----  end of method TrajectoryController_P::update  ----- */


        double	
TrajectoryController_P::limit ( double val )
{
	if (val > u_limit)
	    return u_limit;
	else if (val < l_limit)
	    return l_limit;
	else
	    return val;
}
