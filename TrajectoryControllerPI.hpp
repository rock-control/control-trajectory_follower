/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryControllerPI.hpp
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


#ifndef  TRAJECTORYCONTROLLER_PI_INC
#define  TRAJECTORYCONTROLLER_PI_INC

#include <math.h>
#include <iostream>
#include <eigen2/Eigen/Core>
#include "SimpleIntegrator.hpp"

namespace trajectory_follower{

	/*
	 * =====================================================================================
	 *        Class:  chainedProportionalIntegral
	 *  Description:  
	 * =====================================================================================
	 */
	class chainedProportionalIntegral
	{
		public:
			/* ====================  LIFECYCLE     ======================================= */
			chainedProportionalIntegral ();                             /* constructor */


			/* ====================  ACCESSORS     ======================================= */
			double get_vel_right() {return vel_right;};
			double get_vel_left () {return vel_left; };

			/* ====================  MUTATORS      ======================================= */
			void setConstants (double K0_val, double K2_val, double K3_val, double R_val, double r_val, double samp_time, double ul=7.0, double ll=-7.0);
			Eigen::Vector2d update(double u1, double d_val, double theta_e_val, double c_val, double c_s_val );
		        double limit ( double val );

		protected:

		private:

			double vel_left, vel_right;  // velocities of the left side and right side wheels which is the command to the robot
			double K0, K2, K3; // constant for the controller 

			double R;  // distance between wheels
			double r;  // wheels radius
			double u_limit, l_limit;

			SimpleIntegrator z0;
	}; /* -----  end of class chainedProportionalIntegral  ----- */


}


#endif   /* ----- #ifndef TRAJECTORYCONTROLLER_NOORIENTATION_INC  ----- */
