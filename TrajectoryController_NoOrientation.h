/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryController_NoOrientation.h
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
#include <eigen2/Eigen/Core>

namespace TrajectoryController{

	/*
	 * =====================================================================================
	 *        Class:  TrajectoryController_NoOrientation
	 *  Description:  
	 * =====================================================================================
	 */
	class TrajectoryController_NoOrientation
	{
		public:
			/* ====================  LIFECYCLE     ======================================= */
			TrajectoryController_NoOrientation ();                             /* constructor */
			TrajectoryController_NoOrientation (float l1_val, float K0_val, float R_val, float r_val, double ul=7.0, double ll=-7.0);

			/* ====================  ACCESSORS     ======================================= */
			float k(float theta_e);
			float get_vel_right() {return vel_right;};
			float get_vel_left () {return vel_left; };

			/* ====================  MUTATORS      ======================================= */
			Eigen::Vector2d update(float u1, float d, float theta_e );
        	
   		        double limit ( float val );

		protected:

		private:

			float l1;   // position of reference point P(l1,0) on the robot chassis  such that l1u1 > 0

			float vel_left, vel_right;  // velocities of the left side and right side wheels which is the command to the robot
			float K0; // constant for the calculation of k(d, theta_e)
			float R;  // distance between wheels
			float r;  // wheels radius

			float u_limit, l_limit;

	}; /* -----  end of class TrajectoryController_NoOrientation  ----- */


}

#endif   /* ----- #ifndef TRAJECTORYCONTROLLER_NOORIENTATION_INC  ----- */
