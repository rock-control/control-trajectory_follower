#ifndef TRAJECTORY_FOLLOWER_TYPES_HPP
#define TRAJECTORY_FOLLOWER_TYPES_HPP

#include <base/Float.hpp>
#include <base/Pose.hpp>
#include <base/Time.hpp>
#include <base/commands/Motion2D.hpp>
#include <base/Trajectory.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace trajectory_follower
{

/** Config for finding the reference point in trajectory */
struct TrajectoryConfig
{
    double geometricResolution; ///< Geometric resolution to be used
    double trajectoryFinishDistance; ///< Minimum distance to end point
    ///< of trajectory for considering it
    ///< to be reached
    double splineReferenceError;
    double splineReferenceErrorMarginCoefficient;
    double maxForwardLenght, maxBackwardLenght;

    TrajectoryConfig()
        : geometricResolution( 0.001 ),
          trajectoryFinishDistance( base::unset< double >() ),
          splineReferenceError(base::unset< double >()),
          splineReferenceErrorMarginCoefficient(base::unset< double >()),
          maxForwardLenght( base::unset< double >() ),
          maxBackwardLenght( base::unset< double >() )
    {}
};

/** Follower Status */
enum FollowerStatus
{
    TRAJECTORY_FOLLOWING,
    TRAJECTORY_FINISHED,
    INITIAL_STABILITY_FAILED,
};

/** Controller Types */
enum ControllerType
{
    CONTROLLER_UNKNOWN = -1,
    CONTROLLER_NO_ORIENTATION = 0,
    CONTROLLER_CHAINED = 1,
};

/** No orientation controller config */
struct NoOrientationControllerConfig
{
    double l1; ///< Position of reference point P(l1,0) on the robot chassis such that l1u1 > 0
    double K0; ///< Constant for the calculation of k(d, theta_e)

    double maxRotationalVelocity; ///< Maximum rotational velocity, NaN if no limit is needed

    double pointTurnStart; ///< Angle error at which point turn starts
    double pointTurnEnd;   ///< Angle error at which point turn, once started, stops

    double pointTurnVelocity; ///< Point turn velocity

    NoOrientationControllerConfig()
        : l1( base::unset< double >() ),
          K0( base::unset< double >() ),
          maxRotationalVelocity( base::unset< double >() ),
          pointTurnStart( base::unset< double >() ),
          pointTurnEnd( base::unset< double >() ),
          pointTurnVelocity( base::unset< double >() )
    {}
};

/** Chained controller config */
struct ChainedControllerConfig
{
    double K0; ///< Integrator constant
    double K2; ///< Controller constant
    double K3; ///< Controller constant

    ChainedControllerConfig()
        : K0( base::unset< double >() ),
          K2( base::unset< double >() ),
          K3( base::unset< double >() )
    {}
};

/** Combined config */
struct FollowerConfig
{
    TrajectoryConfig trajectoryConfig; ///< Trajectory configuration
    base::Vector6d poseTransform; ///< Transforms robot pose to pose of the
    ///< robot center of rotation and with
    ///< x-Forward, y-Left, z-Up coordinate system
    ControllerType controllerType; ///< Controller type - CONTROLLER_NO_ORIENTATION
    ///< or CONTROLLER_CHAINED
    NoOrientationControllerConfig noOrientationControllerConfig; ///< Config
    ///< for no_orientation controller
    ChainedControllerConfig chainedControllerConfig; ///< Config for
    ///< Chained controller
};

/** Data for the follower */
struct FollowerData
{
    base::Time time; ///< Time of the last update
    FollowerStatus followerStatus; ///< Status of trajectory follower

    double curveParameter; ///< Current curve parameter
    //base::samples::RigidBodyState referencePose; ///< Reference pose of the robot
    //base::samples::RigidBodyState currentPose; ///< Current pose of the robot
    base::Pose referencePose;
    base::Pose currentPose;

    //base::Trajectory currentTrajectory;

    double referenceHeading; ///< Reference heading
    double currentHeading; ///< Current heading

    double distanceError; ///< Distance Error
    double angleError; ///< Angle error

    base::commands::Motion2D motionCommand; ///< Motion command

    double curveLength; ///< Curve length
    double distanceToEnd; ///< Distance along curve to end

    base::samples::RigidBodyState movementDirection;
    base::Pose lastPose;
    double posError, lastPosError;
    base::Pose goalPose;
    base::samples::RigidBodyState splineSegmentStartPose, splineSegmentEndPose;
    double splineSegmentStartCurveParam, splineSegmentEndCurveParam, splineSegmentGuessCurveParam;
    double distanceMoved;
    double splineReferenceErrorCoefficient;
    double errorMargin;
    base::samples::RigidBodyState splineReferencePose;
    base::samples::RigidBodyState motionCommandViz;
    
    FollowerData()
        : followerStatus( TRAJECTORY_FINISHED ),
          curveParameter( base::unset< double >() ),
          referenceHeading( base::unset< double >() ),
          currentHeading( base::unset< double >() ),
          distanceError( base::unset< double >() ),
          angleError( base::unset< double >() ),
          curveLength( base::unset< double >() ),
          distanceToEnd( base::unset< double >() ),
          posError( base::unset< double >() ),
          lastPosError( base::unset< double >() ),
          splineSegmentStartCurveParam( base::unset< double >() ),
          splineSegmentEndCurveParam( base::unset< double >() ),
          splineSegmentGuessCurveParam( base::unset< double >() ),
          distanceMoved( base::unset< double >() ),
          splineReferenceErrorCoefficient( base::unset< double >() ),
          errorMargin( base::unset< double >() )
    {
        motionCommand.translation = 0.0;
        motionCommand.rotation = 0.0;
        lastPose.position = Eigen::Vector3d(0., 0., 0.);
        lastPose.orientation = Eigen::Quaterniond::Identity();
        splineSegmentStartPose.position = Eigen::Vector3d(0., 0., 0.);
        splineSegmentStartPose.orientation = Eigen::Quaterniond::Identity();
        splineSegmentEndPose.position = Eigen::Vector3d(0., 0., 0.);
        splineSegmentEndPose.orientation = Eigen::Quaterniond::Identity();
        splineReferencePose.position = Eigen::Vector3d(0., 0., 0.);
        splineReferencePose.orientation = Eigen::Quaterniond::Identity();
        motionCommandViz.position = Eigen::Vector3d(0., 0., 0.);
        motionCommandViz.orientation = Eigen::Quaterniond::Identity();
        movementDirection.position = Eigen::Vector3d(0., 0., 0.);
        movementDirection.orientation = Eigen::Quaterniond::Identity();
    }
};

}

#endif