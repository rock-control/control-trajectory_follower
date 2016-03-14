/*
 * =====================================================================================
 *
 *       Filename:  ChainedController.hpp
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

#include "TrajectoryFollowerTypes.hpp"
#include "Motion2D.hpp"

namespace trajectory_follower
{
    /** 
     * Chained Controller class 
     */
    class ChainedController
    {
        public:
            ChainedController();
            ChainedController(const ChainedControllerConfig& config_);

            void configure(const ChainedControllerConfig& config_);
            inline void reset() { z0 = 0.0; }
            const Motion2D& update(double u1, double d, double theta_e, double c, double c_s);
            bool initialStable(double d , double theta_e, double c, double c_max);

        protected:
            ChainedControllerConfig config; ///< Complete config for controller
            bool configured; ///< True if correctly configured
            double z0; ///< Integral term
            Motion2D motionCommand; ///< Motion command 
                                    ///< output of controller
    };
}

#endif