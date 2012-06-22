#ifndef TRAJECTORY_FOLLOWER_TYPES_HPP
#define TRAJECTORY_FOLLOWER_TYPES_HPP

#include <base/pose.h>

namespace trajectory_follower
{
    struct TrajError
    {
	double param;		// Curve parameter
	double d;      		// Distance from curve point
        double theta_e; 	// Heading error

    }; 

    struct RobotPose
    {
	base::Position position;
	double heading;		// Heading angle
    };

    struct CurvePoint 
    {
	double param;		// Curve parameter
	RobotPose   pose;		// Pose on the curve	
    };

}

#endif

