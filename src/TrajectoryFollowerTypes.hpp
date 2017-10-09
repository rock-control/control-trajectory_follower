#ifndef TRAJECTORY_FOLLOWER_TYPES_HPP
#define TRAJECTORY_FOLLOWER_TYPES_HPP

#include <base/Float.hpp>
#include <base/Pose.hpp>
#include <base/Time.hpp>
#include <base/commands/Motion2D.hpp>
#include <base/Trajectory.hpp>
#include <base/samples/RigidBodyState.hpp>

#include "NoOrientationController.hpp"
#include "SamsonController.hpp"
#include "ChainedController.hpp"
#include "SubTrajectory.hpp"

namespace trajectory_follower
{

/** Follower Status */
enum FollowerStatus
{
    TRAJECTORY_FOLLOWING,
    TRAJECTORY_FINISHED,
    INITIAL_STABILITY_FAILED,
    EXEC_TURN_ON_SPOT,
    EXEC_LATERAL,
    SLAM_POSE_CHECK_FAILED
};

/** Controller Types */
enum ControllerType
{
    CONTROLLER_UNKNOWN = -1,
    CONTROLLER_NO_ORIENTATION = 0,
    CONTROLLER_CHAINED = 1,
    CONTROLLER_SAMSON
};

/** Combined config */
struct FollowerConfig
{
    ControllerType controllerType; ///< Controller type - CONTROLLER_NO_ORIENTATION
    ///< or CONTROLLER_CHAINED
    NoOrientationControllerConfig noOrientationControllerConfig; ///< Config
    ///< for no_orientation controller
    ChainedControllerConfig chainedControllerConfig; ///< Config for
    ///< Chained controller
    SamsonControllerConfig samsonControllerConfig;
    
    double dampingAngleUpperLimit;
    double maxRotationalVelocity; ///< Maximum rotational velocity, NaN if no limit is needed
    double pointTurnStart; ///< Angle error at which point turn starts
    double pointTurnEnd;   ///< Angle error at which point turn, once started, stops
    double pointTurnVelocity; ///< Point turn velocity
    double geometricResolution; ///< Geometric resolution to be used
    double trajectoryFinishDistance; ///< Minimum distance to end point
    ///< of trajectory for considering it
    ///< to be reached
    double splineReferenceError;
    double splineReferenceErrorMarginCoefficient;
    double maxForwardLenght, maxBackwardLenght;
    double slamPoseErrorCheckEllipseX, slamPoseErrorCheckEllipseY;
    bool usePoseErrorReachedEndCheck;

    FollowerConfig()
        : controllerType(CONTROLLER_UNKNOWN),
          dampingAngleUpperLimit(base::unset< double >()),
          maxRotationalVelocity(base::unset< double >()),
          pointTurnStart(base::unset< double >()),
          pointTurnEnd(base::unset< double >()),
          pointTurnVelocity(base::unset< double >()),
          geometricResolution(0.001),
          trajectoryFinishDistance(base::unset< double >()),
          splineReferenceError(base::unset< double >()),
          splineReferenceErrorMarginCoefficient(base::unset< double >()),
          maxForwardLenght(base::unset< double >()),
          maxBackwardLenght(base::unset< double >()),
          slamPoseErrorCheckEllipseX(base::unset< double >()),
          slamPoseErrorCheckEllipseY(base::unset< double >()),
          usePoseErrorReachedEndCheck(false)
    {
    }
};

/** Data for the follower */
struct FollowerData
{
    double distanceError;
    double angleError;
    base::samples::RigidBodyState splineReference;
    base::samples::RigidBodyState currentPose;
    std::vector<SubTrajectory> currentTrajectory;
    base::commands::Motion2D cmd;
    base::samples::RigidBodyState splineSegmentStart, splineSegmentEnd;
};

}

#endif
