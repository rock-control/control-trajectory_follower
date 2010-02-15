/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryController_P.h
 *
 *    Description:  Implementation of trajectory controller with orientation control
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


#ifndef  TRAJECTORYCONTROLLER_INC
#define  TRAJECTORYCONTROLLER_INC

#include <math.h>
#include <iostream>
#include <eigen2/Eigen/Core>

namespace TrajectoryController{

	/*
	 * =====================================================================================
	 *        Class:  TrajectoryController_P
	 *  Description:  
	 * =====================================================================================
	 */
	class TrajectoryController_P
	{
		public:
			/* ====================  LIFECYCLE     ======================================= */
			TrajectoryController_P ();                             /* constructor */
			TrajectoryController_P (double K2_val, double K3_val, double R_val, double r_val);

			/* ====================  ACCESSORS     ======================================= */
			double get_vel_right() {return vel_right;};
			double get_vel_left () {return vel_left; };

			/* ====================  MUTATORS      ======================================= */
			Eigen::Vector2d update(double u1, double d_val, double theta_e_val, double c_val, double c_s_val );
		        double limit ( double val );

		protected:

		private:
			double vel_left, vel_right;  // velocities of the left side and right side wheels which is the command to the robot
			double K2, K3; // constant for the controller 

			double R;  // distance between wheels
			double r;  // wheels radius
			double u_limit, l_limit;

	}; /* -----  end of class TrajectoryController_P  ----- */


}


#endif   /* ----- #ifndef TRAJECTORYCONTROLLER_NOORIENTATION_INC  ----- */
