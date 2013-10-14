/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryControllerNoOrientation.hpp
 *
 *    Description:  Implementation of trajectory controller without orientation control
 *    				from 'Springer handbook of robotics' chapter 34 pg 805
 *
 *        Version:  1.0
 *        Created:  10/13/09 10:32:15
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu, ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */


#ifndef  TRAJECTORYCONTROLLER_NOORIENTATION_INC
#define  TRAJECTORYCONTROLLER_NOORIENTATION_INC

#include <math.h>
#include <iostream>
#include <Eigen/Core>

namespace trajectory_follower{

	/*
	 * =====================================================================================
	 *        Class:  noOrientation
	 *  Description:  
	 * =====================================================================================
	 */
	class noOrientation
	{
		public:
			noOrientation ();                             /* constructor */

			double k(double theta_e);
			void setConstants(double l1_val, double K0_val);
	                void setPointTurnSpeed(double val);
			
			Eigen::Vector2d update(double u1, double d, double theta_e );
        	
   			bool checkInstantStability(double u1, double d, double theta_e);
   			bool checkInitialStability(double d , double theta_e, double c_max);
   			
   			inline void setPointTurnUpperLimit(double upper_limit) {
   			    pointTurnUpperLimit = upper_limit;
   			}
   			
   			inline void setPointTurnLowerLimit(double lower_limit) {
   			    pointTurnLowerLimit = lower_limit;
   			}  
   			
   			inline void setRotationalVelocity(double rotational_velocity) {
   			    rotationalVelocity = rotational_velocity;
   			}   			

		protected:

		private:

			double l1;   // position of reference point P(l1,0) on the robot chassis  such that l1u1 > 0
			double K0; // constant for the calculation of k(d, theta_e)
			bool bPointTurn;
            double pointTurnSpeed;
            double rotationalVelocity; // If > 0: defines the upper border of the supported rotational velocity.
            // If the angle between the front axis of the robot and the goal vector 
            // exceeds this limit in radians a point turn will be initiated.
            double pointTurnUpperLimit;  
            double pointTurnLowerLimit; // Limit to stops the point turn. 
	}; /* -----  end of class noOrientation  ----- */


}

#endif   /* ----- #ifndef TRAJECTORYCONTROLLER_NOORIENTATION_INC  ----- */
