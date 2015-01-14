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
    pointTurnSpeed = 0.2;
    rotationalVelocity = 0.0;
    pointTurnUpperLimit = M_PI_2;
    pointTurnLowerLimit = M_PI / 8;
}  /* -----  end of method noOrientation::noOrientation  (constructor)  ----- */


	void 
noOrientation::setConstants(double l1_val, double K0_val)
{
	l1 = l1_val;
	K0 = K0_val;
} 

void noOrientation::setPointTurnSpeed(double val)
{
    pointTurnSpeed = val;
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
        int direction;
        if(u1 >= 0)
           direction = 1.0;
        else
           direction = -1.0;
    	if(checkInstantStability(u1, d, theta_e) && !bPointTurn)
	{
	    u2 = (-u1*tan(theta_e) / (l1*direction) ) - ( (u1 * k(theta_e) * d) / cos(theta_e));
	    // Regard pointTurnSpeed borders.
	    if(rotationalVelocity > 0 && u2 < -rotationalVelocity) {
	        u2 = -rotationalVelocity;
	    }
	    if(rotationalVelocity > 0 && u2 > rotationalVelocity) {
	        u2 = rotationalVelocity;
	    }	    
	    return Eigen::Vector2d(u1, u2);
	}
	else
	{
	    if(!bPointTurn)
	    {
		std::cout << "Robot orientation : OUT OF BOUND.... starting Point-Turn" << std::endl;
		bPointTurn = true;
	    }

	    if(theta_e > pointTurnLowerLimit)
	    {
		u2 = -pointTurnSpeed;
	    }
	    else if(theta_e < -pointTurnLowerLimit)
	    {
		u2 = pointTurnSpeed;
	    }
	    else
	    {	
	        std::cout << "stopped Point-Turn... switching to normal controller" << std::endl;
		bPointTurn = false;
		u2 = 0.0;
	    }
	    
	    return Eigen::Vector2d(0.0,u2);
	}
}		/* -----  end of method noOrientation::update  ----- */


   	bool
noOrientation::checkInitialStability( double d, double theta_e, double c_max)
{
	if( theta_e > -pointTurnUpperLimit && theta_e < pointTurnUpperLimit 
	        && ((l1*c_max)/(1-fabs(d)*c_max)) < 1) {
        return true;
    } else { 
        return false;	    
	}
}

   	bool
noOrientation::checkInstantStability(double u1, double d, double theta_e)
{
	if( theta_e > -pointTurnUpperLimit && theta_e < pointTurnUpperLimit )
        return true;
	else 
	    return false;	    
}
