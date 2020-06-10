#include "SamsonController.hpp"

namespace trajectory_follower {

SamsonController::SamsonController()
    : Controller()
{
    K2 = base::unset<double>();
    K3 = base::unset<double>();
}

SamsonController::SamsonController(const SamsonControllerConfig& config)
    : SamsonController()
{
    if(config.K2 <= 0 || config.K3 <= 0)
    {
        throw std::runtime_error("K2 & K3 value must be greater than zero.");
    }
    
    K2 = config.K2;
    K3 = config.K3;
    configured = true;
}

    
Motion2D& SamsonController::update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature)
{
    if (!configured)
    {
        throw std::runtime_error("controller is not configured.");
    }

    double u1, u2;
    u1 = speed;
    if (speed < 0)
    {
        // Backward motion
        u1 = -speed;
    }

    u2 = -K2*distanceError*u1*(sin(angleError)/angleError) - K3*angleError;

    motionCommand.translation = speed;
    motionCommand.rotation = u2;
    return motionCommand;
}

}