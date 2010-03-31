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
			chainedProportionalIntegral ();                             /* constructor */
			
			void setConstants (double K0_val, double K2_val, double K3_val, double samp_time);
			Eigen::Vector2d update(double u1, double d_val, double theta_e_val, double c_val, double c_s_val );
   			
			bool checkInitialStability(double d , double theta_e, double c, double c_max);

		protected:

		private:

			double K0, K2, K3; // constant for the controller 

			SimpleIntegrator z0;
	}; /* -----  end of class chainedProportionalIntegral  ----- */


}


#endif   /* ----- #ifndef TRAJECTORYCONTROLLER_NOORIENTATION_INC  ----- */
