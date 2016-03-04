/*
 * =====================================================================================
 *
 *       Filename:  NoOrientationController.cpp
 *
 *    Description:  Implementation of trajectory controller without orientation control
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

#include "NoOrientationController.hpp"
#include <base/Logging.hpp>

using namespace trajectory_follower;

NoOrientationController::NoOrientationController()
: pointTurn( false ),
    configured( false )
{
}

NoOrientationController::NoOrientationController( 
        const NoOrientationControllerConfig& config_ )
: pointTurn( false ),
    configured( false )
{
    configure( config_ );
}

void NoOrientationController::configure( 
        const NoOrientationControllerConfig& config_ )
{
    config = config_;
    if( config.l1 <= 0 )
    {
        throw std::runtime_error("l1 value must be greater than zero.");
    }

    if( config.K0 <= 0 )
    {
        throw std::runtime_error("K0 value must be greater than zero.");
    }
    configured = true;
}

const base::commands::Motion2D& NoOrientationController::update( double u1, 
        double d, double theta_e)
{
    if( !configured )
    {
        throw std::runtime_error("NoOrientationController not configured.");
    }

    double u2;
    double direction = 1.0;
    if( u1 < 0 )
    {
        // Backward motion
        direction = -1.0;
        u1 = fabs( u1 );
    }

    if( checkPointTurn( theta_e ) && !pointTurn )
    {
        // No orientation controller ( Page 806 ), Assuming k(d,theta_e)=K0*cos(theta_e)
        u2 = -u1 * ( tan(theta_e) / config.l1 + d * config.K0 );

        if( !base::isUnset< double >( config.maxRotationalVelocity ) )
        {
            ///< Sets limits on rotational velocity
            u2 = std::min( u2,  config.maxRotationalVelocity );
            u2 = std::max( u2, -config.maxRotationalVelocity );
        }        
    }
    else
    {
        if( !pointTurn )
        {
            LOG_INFO_S << "NoOrientationController Robot orientation : "
               "OUT OF BOUND. Starting Point-Turn";
            pointTurn = true;
        }

        if( fabs( theta_e ) > config.pointTurnEnd )
        {
            // Applying velocity in the opposite direction of theta_e
            u2 = copysign( config.pointTurnVelocity, -theta_e );
        }
        else
        {	
            LOG_INFO_S << "NoOrientationController Stopped Point-Turn. "
                "Switching to normal controller";
            pointTurn = false;
            u2 = 0.0;
        }

        u1 = 0;
    }

    motionCommand.translation = u1 * direction;
    motionCommand.rotation = u2;

    return motionCommand;
}

bool NoOrientationController::initialStable( double d, double theta_e, 
        double c_max )
{
    double dist_err_value = ( config.l1 * c_max ) / ( 1 - fabs( d ) * c_max );
    if( theta_e > -M_PI / 2.0 && theta_e < M_PI / 2.0 && dist_err_value < 1 ) 
    {
        return true;
    } 
    else 
    { 
        LOG_INFO_S << "NoOrientationController Initially unstable";
        LOG_INFO_S << "Distance Error: " << d << ", Angle Error: " << theta_e << 
                ", Max Curvature: " << c_max << ", Dist Err: " << dist_err_value;
        return false;	    
    }
}

bool NoOrientationController::checkPointTurn( double theta_e )
{
    if( base::isUnset< double >( config.pointTurnStart ) || 
      base::isUnset< double >( config.pointTurnEnd ) ||
      base::isUnset< double >( config.pointTurnVelocity ) )
    {
        LOG_INFO_S << "NoOrientationController Point turn not set";
        // If point turn start not set dont do point turn
        return false;
    }

    if( theta_e > -config.pointTurnStart && theta_e < config.pointTurnStart )
    {
        return true;
    }
    else 
    {
        return false;
    }
}
