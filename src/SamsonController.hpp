#ifndef TRAJECTORY_FOLLOWER_SAMSON_CONTROLLER
#define TRAJECTORY_FOLLOWER_SAMSON_CONTROLLER

#include "TrajectoryFollowerTypes.hpp"
#include "Motion2D.hpp"

namespace trajectory_follower
{
    /** 
     * Chained Controller class 
     */
    class SamsonController
    {
        public:
            SamsonController();
            SamsonController(const SamsonControllerConfig& config_);

            void configure(const SamsonControllerConfig& config_);
            void reset() { };
            const Motion2D& update(double u1, double d, double theta_e, double c, double c_s);

        protected:
            SamsonControllerConfig config;
            bool configured;
            Motion2D motionCommand;
    };
}

#endif