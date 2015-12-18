#include "TrajectoryFollower.hpp"
#include <base/Logging.hpp>

using namespace Eigen;
using namespace trajectory_follower;
    
double TrajectoryFollower::angleLimit(double angle)
{
    if(angle > M_PI)
	return angle - 2*M_PI;
    else if (angle < -M_PI)
	return angle + 2*M_PI;
    else
     	return angle;
}

TrajectoryFollower::TrajectoryFollower()
    : configured( false ),
    controllerType( CONTROLLER_UNKNOWN )
{
    data.followerStatus = TRAJECTORY_FINISHED;
}

TrajectoryFollower::TrajectoryFollower( const FollowerConfig& followerConfig )
    : configured( false ),
    poseTransform( base::Pose( followerConfig.poseTransform ) ),
    trajectoryConfig( followerConfig.trajectoryConfig ),
    controllerType( followerConfig.controllerType )
{
    data.followerStatus = TRAJECTORY_FINISHED;

    if( controllerType == CONTROLLER_NO_ORIENTATION )
    {
        noOrientationController = NoOrientationController( 
                followerConfig.noOrientationControllerConfig );
    }
    else if( controllerType == CONTROLLER_CHAINED )
    {
        chainedController = ChainedController( 
                followerConfig.chainedControllerConfig );
    }
    else
    {
        throw std::runtime_error("Wrong controller type given, it should be  "
                "either CONTROLLER_NO_ORIENTATION (0) or CONTROLLER_CHAINED (1).");
    }

    configured = true;
}

void TrajectoryFollower::setNewTrajectory( const base::Trajectory &trajectory_,
    const base::Pose& robotPose )
{
    if( !configured )
    {
        throw std::runtime_error("TrajectoryFollower not configured.");
    }

    trajectory = trajectory_;
    trajectory.spline.setGeometricResolution( 
            trajectoryConfig.geometricResolution );
    trajectoryEndPoint = trajectory.spline.getPoint( 
            trajectory.spline.getEndParam() );
    data.curveParameter = trajectory.spline.getStartParam();

    computeErrors( robotPose );
    if( controllerType == CONTROLLER_NO_ORIENTATION )
    {
        noOrientationController.reset();
        if( !noOrientationController.initialStable( data.distanceError, 
                    data.angleError, trajectory.spline.getCurvatureMax() )  ) 
        {
            data.followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }

    }
    else if( controllerType == CONTROLLER_CHAINED )
    {
        chainedController.reset();
        if( !chainedController.initialStable( data.distanceError, 
                    data.angleError, 
                    trajectory.spline.getCurvature( data.curveParameter ),
                    trajectory.spline.getCurvatureMax() ) )
        {
            data.followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }
    }

    data.followerStatus = TRAJECTORY_FOLLOWING;
}
  
void TrajectoryFollower::computeErrors( const base::Pose& robotPose )
{
    data.currentPose.fromTransform( poseTransform.toTransform() * robotPose.toTransform() );
    data.currentHeading = data.currentPose.getYaw();
    if( !trajectory.driveForward() )
    {
        data.currentHeading = angleLimit( data.currentHeading + M_PI );
    }

    if( controllerType == CONTROLLER_NO_ORIENTATION )
    {
        double direction = ( trajectory.driveForward() ? 1 : -1 );
        data.currentPose.position += AngleAxisd( data.currentHeading, Vector3d::UnitZ() )
            * Vector3d( direction * noOrientationController.getConfig().l1, 0, 0);
    }

    Eigen::Vector3d error = trajectory.spline.poseError( data.currentPose.position, 
            data.currentHeading, data.curveParameter );

    data.distanceError = error(0);
    data.angleError = error(1);
    data.curveParameter = error(2);

    data.referenceHeading = trajectory.spline.getHeading( data.curveParameter );
    data.currentPose.position = trajectory.spline.getPoint( data.curveParameter ); 	    
    data.currentPose.orientation = AngleAxisd( data.referenceHeading, Vector3d::UnitZ() );
}

FollowerStatus TrajectoryFollower::traverseTrajectory( 
        base::commands::Motion2D &motionCmd, 
        const base::Pose &robotPose )
{   
    motionCmd.translation = 0;
    motionCmd.rotation = 0;
    if( data.followerStatus != TRAJECTORY_FOLLOWING )
    {
        LOG_INFO_S << "Trajectory follower not active";
        return data.followerStatus;
    }

    computeErrors( robotPose );

    double reachedEnd = false;
    if( base::isUnset< double >( trajectoryConfig.trajectoryFinishDistance ) ) 
    {
        if( ! (data.curveParameter < trajectory.spline.getEndParam()) ) 
        {
            reachedEnd = true;
        }
    }
    else
    {
        if( ( trajectoryEndPoint - data.currentPose.position ).norm() <= 
                trajectoryConfig.trajectoryFinishDistance ||
            !( data.curveParameter < trajectory.spline.getEndParam() ) ) 
        {
            reachedEnd = true;
        }

    }
    if( reachedEnd )
    {
        data.followerStatus = TRAJECTORY_FINISHED;
        return data.followerStatus;
    }

    if( controllerType == CONTROLLER_NO_ORIENTATION )
    {
        motionCmd = noOrientationController.update( trajectory.speed, 
                data.distanceError, data.angleError ); 
    }
    else if( controllerType == CONTROLLER_CHAINED )
    {
        motionCmd = chainedController.update( trajectory.speed, 
                data.distanceError, data.angleError, 
                trajectory.spline.getCurvature( data.curveParameter ),
                trajectory.spline.getVariationOfCurvature( data.curveParameter ));
    }
    
    return data.followerStatus;    
}
