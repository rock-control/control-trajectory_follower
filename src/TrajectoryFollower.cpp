#include "TrajectoryFollower.hpp"
#include <base/Logging.hpp>

#include <limits>

using namespace Eigen;
using namespace trajectory_follower;

double TrajectoryFollower::angleLimit( double angle )
{
    if(angle > M_PI)
        return angle - 2*M_PI;
    else if(angle < -M_PI)
        return angle + 2*M_PI;
    else
        return angle;
}

TrajectoryFollower::TrajectoryFollower()
    : configured(false),
      controllerType(CONTROLLER_UNKNOWN ),
      pointTurn(false),
      pointTurnDirection(1.)
{
    data.followerStatus = TRAJECTORY_FINISHED;
    nearEnd = false;
    data.splineReferenceErrorCoefficient = 0.;
}

TrajectoryFollower::TrajectoryFollower(const FollowerConfig& followerConfig)
    : configured(false),
      poseTransform(base::Pose(followerConfig.poseTransform)),
      trajectoryConfig(followerConfig.trajectoryConfig),
      controllerType(followerConfig.controllerType),
      pointTurn(false),
      pointTurnDirection(1.)
{
    data.followerStatus = TRAJECTORY_FINISHED;
    dampingCoefficient = base::unset< double >();

    // Configures the controller according to controller type
    if(controllerType == CONTROLLER_NO_ORIENTATION) {
        // No orientation controller
        noOrientationController = NoOrientationController(followerConfig.noOrientationControllerConfig);
        controllerConf = followerConfig.noOrientationControllerConfig;
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Chained controller
        chainedController = ChainedController(followerConfig.chainedControllerConfig);
        controllerConf = followerConfig.chainedControllerConfig;
    } else if (controllerType == CONTROLLER_SAMSON) { 
        samsonController = SamsonController(followerConfig.samsonControllerConfig);
        controllerConf = followerConfig.samsonControllerConfig;
    } else {
        throw std::runtime_error("Wrong controller type given, it should be  "
                                 "either CONTROLLER_NO_ORIENTATION (0) or CONTROLLER_CHAINED (1).");
    }

    if (!base::isUnset< double >(controllerConf.dampingAngleUpperLimit))
        dampingCoefficient = 1/std::log(base::Angle::fromRad(controllerConf.dampingAngleUpperLimit).getDeg()+1.);

    configured = true;
    nearEnd = false;

    data.splineReferenceErrorCoefficient = 0.;
    if (!base::isUnset<double>(trajectoryConfig.splineReferenceErrorMarginCoefficient))
        data.splineReferenceErrorCoefficient = trajectoryConfig.splineReferenceErrorMarginCoefficient;
}

void TrajectoryFollower::setNewTrajectory( const base::Trajectory &trajectory_,
        const base::Pose& robotPose )
{
    if (!configured)
        throw std::runtime_error("TrajectoryFollower not configured.");

    // Sets the trajectory
    trajectory = trajectory_;
    nearEnd = false;
    data.goalPose.position = trajectory.spline.getEndPoint();
    data.goalPose.orientation = Eigen::Quaterniond(AngleAxisd(trajectory.spline.getHeading(trajectory.spline.getEndParam()), Eigen::Vector3d::UnitZ()));

    // Sets the geometric resolution
    trajectory.spline.setGeometricResolution(trajectoryConfig.geometricResolution);

    // Curve parameter and length
    data.curveParameter = trajectory.spline.getStartParam();
    data.curveLength = trajectory.spline.getCurveLength(trajectoryConfig.geometricResolution);
    data.distanceToEnd = data.curveLength;
    data.currentPose = robotPose;
    data.lastPose = data.currentPose;
    data.currentHeading = data.currentPose.getYaw();
    data.splineReferencePose.position = data.currentPose.position;
    data.splineReferencePose.orientation = data.currentPose.orientation;
    data.lastPosError = std::numeric_limits< double >::max();
    data.currentTrajectory.clear();
    data.currentTrajectory.push_back(trajectory);
    data.maxCurvature = trajectory.spline.getCurvatureMax();
    data.curvature = trajectory.spline.getCurvature(data.curveParameter);

    // Set state as following if stable
    data.followerStatus = TRAJECTORY_FOLLOWING;

    // Computes the current pose, reference pose and the errors
    computeErrors(robotPose);
    checkTurnOnSpot();

    // Initialize based on controller type
    if (controllerType == CONTROLLER_NO_ORIENTATION) {
        // Resets the controlelr
        noOrientationController.reset();

        // Checks initial stability of the trajectory
        if (!pointTurn && !noOrientationController.initialStable(data.distanceError,
                data.angleError, data.maxCurvature)) {
            data.followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Reset the controller
        chainedController.reset();

        // Checks initial stability of the trajectory
        if (!pointTurn && !chainedController.initialStable(data.distanceError, data.angleError,
                data.curvature, data.maxCurvature) ) {
            data.followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }
    } else if (controllerType == trajectory_follower::CONTROLLER_SAMSON) {
        samsonController.reset();
    }
}

void TrajectoryFollower::computeErrors(const base::Pose& robotPose)
{
    data.lastPose = data.currentPose;

    // Transform robot pose into pose of the center of rotation
    data.currentPose.fromTransform(robotPose.toTransform() * poseTransform.toTransform());

    // Gets the heading of the current pose
    data.currentHeading = data.currentPose.getYaw();

    // Change heading based on direction of motion
    if(!trajectory.driveForward())
        data.currentHeading = angleLimit(data.currentHeading + M_PI);

    Vector2d movementVector = data.currentPose.position.head(2) - data.lastPose.position.head(2);
    double distanceMoved = movementVector.norm();
    double movementDirection = atan2(movementVector.y(), movementVector.x());

    double direction = 1.;
    if (std::abs(movementDirection - data.currentHeading) > base::Angle::fromDeg(90).getRad())
        direction = -1.;

    double errorMargin = distanceMoved*data.splineReferenceErrorCoefficient;
    if (!base::isUnset<double>(trajectoryConfig.splineReferenceError))
        errorMargin += std::abs(trajectoryConfig.splineReferenceError);

    //Find upper and lower bound for local search
    double forwardLength, backwardLength;
    forwardLength = distanceMoved + errorMargin;
    backwardLength = forwardLength;

    if (!base::isUnset<double>(trajectoryConfig.maxForwardLenght))
        forwardLength = std::min(forwardLength, trajectoryConfig.maxForwardLenght);

    if (!base::isUnset<double>(trajectoryConfig.maxBackwardLenght))
        backwardLength = std::min(backwardLength, trajectoryConfig.maxBackwardLenght);

    double splineSegmentStartCurveParam, splineSegmentEndCurveParam, splineSegmentGuessCurveParam;
    splineSegmentStartCurveParam = trajectory.spline.getStartParam();
    splineSegmentEndCurveParam = trajectory.spline.getEndParam();
    splineSegmentGuessCurveParam = data.curveParameter;

    if (trajectory.spline.length(trajectory.spline.getStartParam(), data.curveParameter, trajectoryConfig.geometricResolution) > backwardLength)
        splineSegmentStartCurveParam = trajectory.spline.advance(data.curveParameter, -backwardLength, trajectoryConfig.geometricResolution).first;

    if (data.distanceToEnd > forwardLength)
        splineSegmentEndCurveParam = trajectory.spline.advance(data.curveParameter, forwardLength, trajectoryConfig.geometricResolution).first;

    double dist = distanceMoved*direction;
    if ((dist > 0. && data.distanceToEnd > dist) || (dist < 0. && (data.curveLength-data.distanceToEnd) > std::abs(dist)))
        splineSegmentGuessCurveParam = trajectory.spline.advance(data.curveParameter, dist, trajectoryConfig.geometricResolution).first;
    else if (dist > 0.)
        splineSegmentGuessCurveParam = trajectory.spline.getEndParam();
    else
        splineSegmentGuessCurveParam = trajectory.spline.getStartParam();

    Eigen::Vector3d curPos(data.currentPose.position);
    curPos.z() = 0;

    data.curveParameter = trajectory.spline.localClosestPointSearch(curPos, splineSegmentGuessCurveParam, splineSegmentStartCurveParam, splineSegmentEndCurveParam, trajectoryConfig.geometricResolution);

    // Setting reference values
    data.referenceHeading = trajectory.spline.getHeading(data.curveParameter);

    data.curvature = trajectory.spline.getCurvature(data.curveParameter);
    data.variationOfCurvature = trajectory.spline.getVariationOfCurvature(data.curveParameter);
    data.referencePose.position = trajectory.spline.getPoint(data.curveParameter);
    data.referencePose.orientation = AngleAxisd(data.referenceHeading, Vector3d::UnitZ());
    data.angleError = angleLimit(trajectory.spline.headingError(data.currentHeading, data.curveParameter));
    data.distanceError = trajectory.spline.distanceError(data.currentPose.position, data.curveParameter);

    if (controllerType == trajectory_follower::CONTROLLER_NO_ORIENTATION
            && !base::isUnset<double>(noOrientationController.getConfig().l1))
    {
        double targetCurveParam = data.curveParameter;
        base::Pose targetPose = data.currentPose;
        double targetCurveParamCoefficient = (data.maxCurvature-data.variationOfCurvature)/data.maxCurvature;
        targetCurveParamCoefficient = std::max(1., targetCurveParamCoefficient);
        targetCurveParamCoefficient = std::min(targetCurveParamCoefficient, 0.1);
        targetCurveParam = trajectory.spline.advance(targetCurveParam, targetCurveParamCoefficient*noOrientationController.getConfig().l1, trajectoryConfig.geometricResolution).first;
        targetPose.position += AngleAxisd(data.currentHeading, Vector3d::UnitZ()) * Vector3d(noOrientationController.getConfig().l1, 0, 0);
        Eigen::Vector3d fwPosition = targetPose.position;
        fwPosition.z() = 0.;
        double endParam = std::min(trajectory.spline.getEndParam(), splineSegmentEndCurveParam + noOrientationController.getConfig().l1);
        targetCurveParam = trajectory.spline.localClosestPointSearch(fwPosition, targetCurveParam, splineSegmentStartCurveParam, endParam, trajectoryConfig.geometricResolution);

        if (noOrientationController.getConfig().useForwardAngleError)
        {
            data.angleError = angleLimit(trajectory.spline.headingError(data.currentHeading, targetCurveParam));
            data.referencePose.orientation = AngleAxisd(trajectory.spline.getHeading(targetCurveParam), Vector3d::UnitZ());
        }

        if (noOrientationController.getConfig().useForwardDistanceError)
        {
            data.distanceError = trajectory.spline.distanceError(targetPose.position, targetCurveParam);
            data.referencePose.position = trajectory.spline.getPoint(targetCurveParam);
        }
    }
}

FollowerStatus TrajectoryFollower::traverseTrajectory(
    base::commands::Motion2D &motionCmd,
    const base::Pose &robotPose )
{
    motionCmd.translation = 0;
    motionCmd.rotation = 0;

    // Return if there is no trajectory to follow
    if(data.followerStatus != TRAJECTORY_FOLLOWING && data.followerStatus != TURN_ON_SPOT) {
        LOG_INFO_S << "Trajectory follower not active";
        return data.followerStatus;
    }

    data.time = base::Time::now();

    // Computes reference pose and the errors
    computeErrors(robotPose);

    data.splineReferencePose.position = data.referencePose.position;
    data.splineReferencePose.orientation = data.referencePose.orientation;

    // Finding reached end condition
    bool reachedEnd = false;

    data.distanceToEnd = trajectory.spline.getCurveLength(data.curveParameter, trajectoryConfig.geometricResolution);
    data.lastPosError = data.posError;
    data.posError = (robotPose.position.head(2) - data.goalPose.position.head(2)).norm();

    // If distance to trajectory finish set
    if (base::isUnset<double>(trajectoryConfig.trajectoryFinishDistance)) {
        // Only curve parameter
        if (!(data.curveParameter < trajectory.spline.getEndParam()))
            reachedEnd = true;
    } else {
        // Distance along curve to end point
        if (data.distanceToEnd <= trajectoryConfig.trajectoryFinishDistance)
        {
            if (trajectoryConfig.usePoseErrorReachedEndCheck)
                nearEnd = true;
            else
                reachedEnd = true;
        }

        if (data.posError <= trajectoryConfig.trajectoryFinishDistance)
            reachedEnd = true;
    }

    if (nearEnd) {
        if (data.posError > data.lastPosError)
            reachedEnd = true;
    }

    // If end reached
    if (reachedEnd) {
        // Trajectory finished
        nearEnd = false;
        LOG_INFO_S << "Trajectory follower finished";
        data.followerStatus = TRAJECTORY_FINISHED;
        return data.followerStatus;
    }

    checkTurnOnSpot();

    if (pointTurn) {
        if (data.angleError < -controllerConf.pointTurnEnd
                || data.angleError > controllerConf.pointTurnEnd)
        {
            motionCmd.rotation = pointTurnDirection * controllerConf.pointTurnVelocity;
        }
        else
        {
            std::cout << "stopped Point-Turn. Switching to normal controller" << std::endl;
            pointTurn = false;
            data.followerStatus = TRAJECTORY_FOLLOWING;
            pointTurnDirection = 1.;
        }
    } else {
        // If trajectory follower running call the controller based on the
        // controller type
        if (controllerType == CONTROLLER_NO_ORIENTATION) {
            // No orientation controller update
            motionCmd = noOrientationController.update(trajectory.speed, data.distanceError, data.angleError);
        } else if (controllerType == CONTROLLER_CHAINED) {
            // Chained controller update
            motionCmd = chainedController.update(trajectory.speed, data.distanceError, data.angleError,
                                                 data.curvature, data.variationOfCurvature);
        } else if (controllerType == CONTROLLER_SAMSON) {
            motionCmd = samsonController.update(trajectory.speed, data.distanceError, data.angleError,
                                                data.curvature, data.variationOfCurvature);
        }

        while (motionCmd.rotation > 2*M_PI || motionCmd.rotation < -2*M_PI)
            motionCmd.rotation += (motionCmd.rotation > 2*M_PI ? -1 : 1)*2*M_PI;

        // HACK: use damping factor to prevend oscillating steering behavior
        if (!base::isUnset<double>(controllerConf.dampingAngleUpperLimit) && controllerConf.dampingAngleUpperLimit > 0) {
            data.dampingFactor = std::min(1., std::log(std::abs(base::Angle::fromRad(motionCmd.rotation).getDeg())+1.)*dampingCoefficient);
            motionCmd.rotation *= data.dampingFactor;
        }
    }

    data.motionCommand = motionCmd;
    return data.followerStatus;
}

void TrajectoryFollower::checkTurnOnSpot()
{
    if (pointTurn)
        return;

    if(!(data.angleError > -controllerConf.pointTurnStart && data.angleError < controllerConf.pointTurnStart))
    {
        std::cout << "robot orientation : OUT OF BOUND ["  << data.angleError << ", " << controllerConf.pointTurnStart << "]. starting point-turn" << std::endl;
        pointTurn = true;
        data.followerStatus = TURN_ON_SPOT;

        if (data.angleError < -controllerConf.pointTurnStart)
            pointTurnDirection = -1.;
    }
}