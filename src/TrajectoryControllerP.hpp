/*
 * =====================================================================================
 *
 *       Filename:  TrajectoryControllerP.hpp
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


#ifndef  TRAJECTORYCONTROLLER_P_INC
#define  TRAJECTORYCONTROLLER_P_INC

#include <math.h>
#include <iostream>
#include <Eigen/Core>

namespace trajectory_follower{

	/*
	 * =====================================================================================
	 *        Class:  chainedProportional
	 *  Description:  
	 * =====================================================================================
	 */
	class chainedProportional
	{
		public:
			chainedProportional ();                             /* constructor */


			void setConstants (double K2_val, double K3_val);
			Eigen::Vector2d update(double u1, double d_val, double theta_e_val, double c_val, double c_s_val );
   			bool checkInitialStability(double d , double theta_e, double c, double c_max);

		protected:

		private:
			double K2, K3; // constant for the controller 
	}; /* -----  end of class chainedProportional  ----- */


}


#endif   /* ----- #ifndef TRAJECTORYCONTROLLER_NOORIENTATION_INC  ----- */
