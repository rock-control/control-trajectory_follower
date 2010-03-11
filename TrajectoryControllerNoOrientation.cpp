/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryControllerNoOrientation.cpp
 *
 *    Description:  Implementation of trajectory controller without orientation control
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

#include "TrajectoryControllerNoOrientation.hpp"

using namespace trajectory_follower;

noOrientation::noOrientation ()
{
    bPointTurn = false;
}  /* -----  end of method noOrientation::noOrientation  (constructor)  ----- */


	void 
noOrientation::setConstants(double l1_val, double K0_val, double R_val, double r_val, double ul, double ll)
{
	l1 = l1_val;
	K0 = K0_val;

	R = R_val;
	r = r_val;

	u_limit = ul; // radians per second
	l_limit = ll; // radians per second 
} 


	double
noOrientation::k (double theta_e )
{
	// k(d,theta_e) = 0 * cos(theta_e)
	return K0 * cos(theta_e);
}		/* -----  end of method noOrientation::k  ----- */


        Eigen::Vector2d	
noOrientation::update (double u1, double d, double theta_e )
{
        double u2;
    	if(checkInstantStability(u1, d, theta_e) && !bPointTurn)
	{
	    u2 = (-u1*tan(theta_e) / l1) - ( (u1 * k(theta_e) * d) / cos(theta_e));

	    vel_right = limit((u1 + R*u2) / r);
	    vel_left  = limit((u1 - R*u2) / r);	
	    
	    return Eigen::Vector2d(u1, u2);
	}
	else
	{
	    std::cout << "Robot orientation : OUT OF BOUND.... starting Point-Turn" << std::endl;
	    bPointTurn = true;

	    if(theta_e > M_PI / 8)
		u2 = -M_PI;
	    else if(theta_e < -M_PI / 8)
		u2 = M_PI;
	    else 
		bPointTurn = false;

	    vel_right = limit((R*u2) / r);
	    vel_left  = limit((R*u2) / r);	
	    
	    return Eigen::Vector2d(0.0,u2);
	}
}		/* -----  end of method noOrientation::update  ----- */


        double	
noOrientation::limit ( double val )
{
	if (val > u_limit)
	    return u_limit;
	else if (val < l_limit)
	    return l_limit;
	else
	    return val;
}

   	bool
noOrientation::checkInitialStability( double d, double theta_e, double c_max)
{
	if( theta_e > -M_PI_2 && theta_e < M_PI_2 && ((l1*c_max)/(1-fabs(d)*c_max)) < 1)
    	    return true;
	else 
	    return false;	    
}

   	bool
noOrientation::checkInstantStability(double u1, double d, double theta_e)
{
	if( u1*l1 >= 0.0 && theta_e > -M_PI_2 && theta_e < M_PI_2 )
    	    return true;
	else 
	    return false;	    
}
