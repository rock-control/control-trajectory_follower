#include "ChainedController.hpp"

namespace trajectory_follower {

ChainedController::ChainedController()
    : Controller()
{
    K0 = base::unset<double>();
    K2 = base::unset<double>();
    K3 = base::unset<double>();
}

ChainedController::ChainedController(const ChainedControllerConfig& config)
    : ChainedController()
{
    if (config.K2 <= 0 || config.K3 <= 0)
    {
        throw std::runtime_error("K2 & K3 value must be greater than zero.");
    }

    if (config.K0 <= 0 || base::isUnset< double >(config.K0))
    {
        std::cout << "ChainedController disabling integral" << std::endl;
        // Disabling integral
        K0 = 0;
    }
    else
    {
        K0 = config.K0;
    }

    K2 = config.K2;
    K3 = config.K3;
    configured = true;
}

    
Motion2D& ChainedController::update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature)
{
    if (!configured)
    {
        throw std::runtime_error("controller is not configured.");
    }

    double d_dot, s_dot, z2, z3, v1, v2, u1, u2;
    u1 = speed;

    double direction = 1.0;
    if(speed < 0) {
        // Backward motion
        direction = -1.0;
        u1 = fabs(u1);
    }

    d_dot = u1 * sin(angleError);
    s_dot = u1 * cos(angleError) / (1.0-distanceError*curvature);

    v1 = s_dot;
    z2 = distanceError;
    z3 = (1.0-(distanceError*curvature))*tan(angleError);

    controllerIntegral += (s_dot * z2);
    v2 = -fabs(v1)*K0*controllerIntegral - v1*K2*z2 - fabs(v1)*K3 * z3;
    u2 = (v2 + (d_dot*curvature + distanceError*variationOfCurvature*s_dot)*tan(angleError))
          /((1.0-distanceError*curvature)*(1+pow(tan(angleError),2))) + s_dot*curvature;

    motionCommand.translation = u1*direction;
    motionCommand.rotation = u2;
    return motionCommand;
}

}