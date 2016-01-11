/*
 * =====================================================================================
 *
 *       Filename:  ChainedController.cpp
 *
 *    Description:  Implementation of trajectory controller with orientation control
 *    				from 'Springer handbook of robotics' chapter 34 pg 805
 *
 *        Version:  1.0
 *        Created:  10/13/09 10:30:32
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu, ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#include "ChainedController.hpp"
#include <base/Logging.hpp>

using namespace trajectory_follower;

ChainedController::ChainedController()
: configured( false ),
    z0( 0 )
{
}

ChainedController::ChainedController( const ChainedControllerConfig& config_ )
: configured( false ),
    z0( 0 )
{
    configure( config_ );
}

void ChainedController::configure( const ChainedControllerConfig& config_ )
{
    config = config_;
    if( config.K2 <= 0 || config.K3 <= 0 )
    {
        throw std::runtime_error("K2 & K3 value must be greater than zero.");
    }

    if( config.K0 <= 0 || base::isUnset< double >( config.K0 ) )
    {
        LOG_INFO_S << "ChainedController disabling integral";
        // Disabling integral
        config.K0 = 0;
    }
    configured = true;
    reset();
}

const base::commands::Motion2D& ChainedController::update( double u1, 
        double d, double theta_e, double c, double c_s )
{
    double d_dot, s_dot, z2, z3, v1, v2, u2;
    
    double direction = 1.0;
    if( u1 < 0 )
    {
        // Backward motion
        direction = -1.0;
        u1 = fabs( u1 );
    }

    d_dot = u1 * sin(theta_e);
    s_dot = u1 * cos(theta_e) / (1.0-d*c);

    z2 = d;
    z3 = (1.0-(d*c))*tan(theta_e);

    v1 = u1 * cos(theta_e) / (1.0 - d*c);

    z0 += (v1 * z2);

    v2 = -(fabs(v1)) * config.K0 * z0 + (-v1 * config.K2 * z2) - 
        (fabs(v1) * config.K3 * z3);

    u2 = ((v2 + ((d_dot*c + d*c_s*s_dot)*tan(theta_e))) / 
            ((1.0-d*c)*(1+pow(tan(theta_e),2)))) + (s_dot*c);

    motionCommand.translation = u1 * direction;
    motionCommand.rotation = u2;

    return motionCommand;
}

bool ChainedController::initialStable( double d, double theta_e, double c, 
        double c_max )
{
    // HURWITZ condition should also be added 
    double z2, z3;
    z2 = d;
    z3 = (1.0-(d*c_max))*tan(theta_e);

    if( z2*z2+(z3*z3/(config.K2-(config.K0/config.K3))) < (1/(c_max*c_max)) )
    {
        return true;
    }
    else 
    {
        LOG_INFO_S << "ChainedController Initially unstable";
        return false;	    
    }
}
