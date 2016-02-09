/*
 * =====================================================================================
 *
 *       Filename:  NoOrientationController.hpp
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

#include "TrajectoryFollowerTypes.hpp"

namespace trajectory_follower
{
    /** 
     * No Orientation Controller class 
     */
    class NoOrientationController
    {
        public:
            /** Constuctor */
            NoOrientationController();
            NoOrientationController( 
                    const NoOrientationControllerConfig& config_ );

            inline const NoOrientationControllerConfig& getConfig() 
            { return config; }

            void configure( const NoOrientationControllerConfig& config_ );
            inline void reset() {}

            bool initialStable( double d , double theta_e, double c_max );
            const base::commands::Motion2D& update( double u1, double d, double theta_e );

        protected:
            bool checkPointTurn( double theta_e);

            NoOrientationControllerConfig config; ///< Complete config for controller
            bool pointTurn; ///< True if doing point turn
            bool configured; ///< True if correctly configured
            base::commands::Motion2D motionCommand; ///< Motion command output of controller
    };
}

#endif
