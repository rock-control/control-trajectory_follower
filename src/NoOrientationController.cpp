#include "NoOrientationController.hpp"

using namespace trajectory_follower;
    
NoOrientationController::NoOrientationController()
    : Controller()
{
    l1 = base::unset<double>();
    K0 = base::unset<double>();
}

NoOrientationController::NoOrientationController(const NoOrientationControllerConfig& config)
    : NoOrientationController()
{
    if (config.l1 <= 0)
    {
        throw std::runtime_error("l1 value must be greater than zero.");
    }

    if (config.K0 <= 0)
    {
        throw std::runtime_error("K0 value must be greater than zero.");
    }

    l1 = config.l1;
    K0 = config.K0;
    configured = true;
}

Motion2D& NoOrientationController::update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature)
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

    // No orientation controller ( Page 806 ), Assuming k(d,theta_e)=K0*cos(theta_e)
    u2 = -u1 * (tan(angleError) / l1 + distanceError * K0);

    motionCommand.translation = speed;
    motionCommand.rotation = u2;
    return motionCommand;
}

