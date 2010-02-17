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
#include <eigen2/Eigen/Core>

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
			/* ====================  LIFECYCLE     ======================================= */
			noOrientation ();                             /* constructor */

			/* ====================  ACCESSORS     ======================================= */
			double k(double theta_e);
			double get_vel_right() {return vel_right;};
			double get_vel_left () {return vel_left; };

			/* ====================  MUTATORS      ======================================= */
			void setConstants(double l1_val, double K0_val, double R_val, double r_val, double ul=7.0, double ll=-7.0);
			Eigen::Vector2d update(double u1, double d, double theta_e );
        	
   		        double limit ( double val );

		protected:

		private:

			double l1;   // position of reference point P(l1,0) on the robot chassis  such that l1u1 > 0

			double vel_left, vel_right;  // velocities of the left side and right side wheels which is the command to the robot
			double K0; // constant for the calculation of k(d, theta_e)
			double R;  // distance between wheels
			double r;  // wheels radius

			double u_limit, l_limit;

	}; /* -----  end of class noOrientation  ----- */


}

#endif   /* ----- #ifndef TRAJECTORYCONTROLLER_NOORIENTATION_INC  ----- */
