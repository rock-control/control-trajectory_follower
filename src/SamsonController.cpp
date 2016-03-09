#include "SamsonController.hpp"

using namespace trajectory_follower;

SamsonController::SamsonController()
: configured( false )
{
}

SamsonController::SamsonController(const SamsonControllerConfig& config_)
: configured( false )
{
    configure(config_);
}

void SamsonController::configure(const SamsonControllerConfig& config_)
{
    config = config_;
    if( config.K2 <= 0 || config.K3 <= 0 )
    {
        throw std::runtime_error("K2 & K3 value must be greater than zero.");
    }
    
    configured = true;
    reset();
}

const base::commands::Motion2D& SamsonController::update( double u1, 
        double d, double theta_e, double c, double c_s )
{
    double u2;
    
    double direction = 1.0;
    if(u1 < 0) {
        // Backward motion
        direction = -1.0;
        u1 = fabs(u1);
    }

    u2 = -config.K2*d*u1*(sin(theta_e)/theta_e) - config.K3*theta_e;
    
    motionCommand.translation = u1*direction;
    motionCommand.rotation = u2;

    return motionCommand;
}