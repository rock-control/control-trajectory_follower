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
    bool usePoseErrorReachedEndCheck;

    TrajectoryConfig()
        : geometricResolution( 0.001 ),
          trajectoryFinishDistance( base::unset< double >() ),
          splineReferenceError(base::unset< double >()),
          splineReferenceErrorMarginCoefficient(base::unset< double >()),
          maxForwardLenght( base::unset< double >() ),
          maxBackwardLenght( base::unset< double >() )
    {
        usePoseErrorReachedEndCheck = false;
    }
};

/** Follower Status */
enum FollowerStatus
{
    TRAJECTORY_FOLLOWING,
    TRAJECTORY_FINISHED,
    INITIAL_STABILITY_FAILED,
    TURN_ON_SPOT
};

/** Controller Types */
enum ControllerType
{
    CONTROLLER_UNKNOWN = -1,
    CONTROLLER_NO_ORIENTATION = 0,
    CONTROLLER_CHAINED = 1,
    CONTROLLER_SAMSON
};

struct ControllerConfig
{
    double dampingAngleUpperLimit;
    double maxRotationalVelocity; ///< Maximum rotational velocity, NaN if no limit is needed
    double pointTurnStart; ///< Angle error at which point turn starts
    double pointTurnEnd;   ///< Angle error at which point turn, once started, stops
    double pointTurnVelocity; ///< Point turn velocity

    ControllerConfig()
        : dampingAngleUpperLimit(base::unset< double >())
        , maxRotationalVelocity(base::unset< double >())
        , pointTurnStart(base::unset< double >())
        , pointTurnEnd(base::unset< double >())
        , pointTurnVelocity(base::unset< double >())
    {
    }
};

/** No orientation controller config */
struct NoOrientationControllerConfig : public ControllerConfig
{
    double l1; ///< Position of reference point P(l1,0) on the robot chassis such that l1u1 > 0
    double K0; ///< Constant for the calculation of k(d, theta_e)
    bool useForwardAngleError, useForwardDistanceError;

    NoOrientationControllerConfig()
        : ControllerConfig()
    {
        l1 = base::unset< double >();
        K0 = base::unset< double >();
	useForwardAngleError = false;
	useForwardDistanceError = false;
    }

    void operator = (const ControllerConfig& base_)
    {
        ControllerConfig::operator=(base_);
    }
};

/** Chained controller config */
struct ChainedControllerConfig : public ControllerConfig
{
    double K0; ///< Integrator constant
    double K2; ///< Controller constant
    double K3; ///< Controller constant

    ChainedControllerConfig()
        : ControllerConfig()
    {
        K0 = base::unset< double >();
        K2 = base::unset< double >();
        K3 = base::unset< double >();
    }
    
    void operator = (const ControllerConfig& base_)
    {
        ControllerConfig::operator=(base_);
    }
};

struct SamsonControllerConfig : public ControllerConfig
{
    double K2;
    double K3;

    SamsonControllerConfig()
        : ControllerConfig()
    {
        K2 = base::unset< double >();
        K3 = base::unset< double >();
    }
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
    SamsonControllerConfig samsonControllerConfig;
};

/** Data for the follower */
struct FollowerData
{
    base::Time time; ///< Time of the last update
    FollowerStatus followerStatus; ///< Status of trajectory follower
    double curveParameter; ///< Current curve parameter
    base::Pose referencePose;
    base::Pose currentPose;
    double referenceHeading; ///< Reference heading
    double currentHeading; ///< Current heading
    double distanceError; ///< Distance Error
    double angleError; ///< Angle error
    base::commands::Motion2D motionCommand; ///< Motion command
    double curveLength; ///< Curve length
    double distanceToEnd; ///< Distance along curve to end
    base::Pose lastPose;
    double posError, lastPosError;
    base::Pose goalPose;
    double splineReferenceErrorCoefficient;
    base::samples::RigidBodyState splineReferencePose;
    double curvature, variationOfCurvature, maxCurvature;
    std::vector< base::Trajectory > currentTrajectory;
    double dampingFactor;

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
          splineReferenceErrorCoefficient( base::unset< double >() ),
          curvature(base::unset< double >()),
          variationOfCurvature(base::unset< double >()),
          maxCurvature(base::unset< double >()),
          dampingFactor(base::unset< double >())
    {
        motionCommand.translation = 0.0;
        motionCommand.rotation = 0.0;
        lastPose.position = Eigen::Vector3d(0., 0., 0.);
        lastPose.orientation = Eigen::Quaterniond::Identity();
        splineReferencePose.position = Eigen::Vector3d(0., 0., 0.);
        splineReferencePose.orientation = Eigen::Quaterniond::Identity();
    }
};

}

#endif