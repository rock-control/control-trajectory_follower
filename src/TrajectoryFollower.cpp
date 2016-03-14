#include "TrajectoryFollower.hpp"
#include <base/Logging.hpp>

#include <limits>

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
    followerStatus = TRAJECTORY_FINISHED;
    nearEnd = false;
    splineReferenceErrorCoefficient = 0.;
}

TrajectoryFollower::TrajectoryFollower(const FollowerConfig& followerConfig)
    : configured(false),
      controllerType(followerConfig.controllerType),
      followerConf(followerConfig),
      pointTurn(false),
      pointTurnDirection(1.)
{
    followerStatus = TRAJECTORY_FINISHED;
    dampingCoefficient = base::unset< double >();

    // Configures the controller according to controller type
    if(controllerType == CONTROLLER_NO_ORIENTATION) {
        // No orientation controller
        noOrientationController = NoOrientationController(followerConfig.noOrientationControllerConfig);
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Chained controller
        chainedController = ChainedController(followerConfig.chainedControllerConfig);
    } else if (controllerType == CONTROLLER_SAMSON) {
        samsonController = SamsonController(followerConfig.samsonControllerConfig);
    } else {
        throw std::runtime_error("Wrong controller type given, it should be  "
                                 "either CONTROLLER_NO_ORIENTATION (0) or CONTROLLER_CHAINED (1).");
    }

    if (!base::isUnset< double >(followerConf.dampingAngleUpperLimit))
        dampingCoefficient = 1/std::log(base::Angle::fromRad(followerConf.dampingAngleUpperLimit).getDeg()+1.);

    configured = true;
    nearEnd = false;

    splineReferenceErrorCoefficient = 0.;
    if (!base::isUnset<double>(followerConf.splineReferenceErrorMarginCoefficient))
        splineReferenceErrorCoefficient = followerConf.splineReferenceErrorMarginCoefficient;
}

void TrajectoryFollower::setNewTrajectory(const SubTrajectory &trajectory, const base::Pose& robotPose)
{
    if (!configured)
        throw std::runtime_error("TrajectoryFollower not configured.");

    // Sets the trajectory
    this->trajectory = trajectory;
    nearEnd = false;

    // Sets the geometric resolution
    this->trajectory.setGeometricResolution(followerConf.geometricResolution);

    // Curve parameter and length
    currentPose = robotPose;
    lastPose = currentPose;
    lastPosError = lastAngleError =std::numeric_limits< double >::max();
    currentCurveParameter = this->trajectory.getStartParam();
    distanceError = angleError = 0.;
    posError = lastPosError;

    followerData.currentPose.position = currentPose.position;
    followerData.currentPose.orientation = currentPose.orientation;
    base::Pose2D refPose = this->trajectory.getIntermediatePoint(currentCurveParameter);
    followerData.splineReference.position = Eigen::Vector3d(refPose.position.x(), refPose.position.y(), 0.);
    followerData.splineReference.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(refPose.orientation, Eigen::Vector3d::UnitZ()));
    followerData.currentTrajectory.clear();
    followerData.currentTrajectory.push_back(this->trajectory.toBaseTrajectory());

    // Computes the current pose, reference pose and the errors
    computeErrors(robotPose);

    followerData.angleError = angleError;
    followerData.distanceError = distanceError;

    followerStatus = TRAJECTORY_FOLLOWING;
    if (trajectory.driveMode == ModeDiagonal)
        followerStatus = EXEC_LATERAL;

    // Initialize based on controller type
    if (controllerType == CONTROLLER_NO_ORIENTATION) {
        // Resets the controlelr
        noOrientationController.reset();

        // Checks initial stability of the trajectory
        if (!checkTurnOnSpot() && !noOrientationController.initialStable(distanceError,
                angleError, this->trajectory.getCurvatureMax())) {
            followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Reset the controller
        chainedController.reset();

        // Checks initial stability of the trajectory
        if (!checkTurnOnSpot() && !chainedController.initialStable(distanceError, angleError,
                this->trajectory.getCurvature(currentCurveParameter), this->trajectory.getCurvatureMax()) ) {
            followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }
    } else if (controllerType == trajectory_follower::CONTROLLER_SAMSON) {
        samsonController.reset();
    }
}

void TrajectoryFollower::computeErrors(const base::Pose& robotPose)
{
    lastPose = currentPose;
    currentPose = robotPose;

    // Gets the heading of the current pose
    double currentHeading = currentPose.getYaw();

    // Change heading based on direction of motion
    if(!trajectory.driveForward())
        currentHeading = angleLimit(currentHeading + M_PI);

    Eigen::Vector2d movementVector = currentPose.position.head(2) - lastPose.position.head(2);
    double distanceMoved = movementVector.norm();
    double movementDirection = atan2(movementVector.y(), movementVector.x());

    double direction = 1.;
    if (std::abs(movementDirection - currentHeading) > base::Angle::fromDeg(90).getRad())
        direction = -1.;

    double errorMargin = distanceMoved*splineReferenceErrorCoefficient;
    if (!base::isUnset<double>(followerConf.splineReferenceError))
        errorMargin += std::abs(followerConf.splineReferenceError);

    //Find upper and lower bound for local search
    double forwardLength, backwardLength;
    forwardLength = distanceMoved + errorMargin;
    backwardLength = forwardLength;

    if (!base::isUnset<double>(followerConf.maxForwardLenght))
        forwardLength = std::min(forwardLength, followerConf.maxForwardLenght);

    if (!base::isUnset<double>(followerConf.maxBackwardLenght))
        backwardLength = std::min(backwardLength, followerConf.maxBackwardLenght);

    double dist = distanceMoved*direction;
    double splineSegmentStartCurveParam, splineSegmentEndCurveParam, splineSegmentGuessCurveParam;
    splineSegmentStartCurveParam = trajectory.advance(currentCurveParameter, -backwardLength);
    splineSegmentEndCurveParam = trajectory.advance(currentCurveParameter, forwardLength);
    splineSegmentGuessCurveParam = trajectory.advance(currentCurveParameter, dist);

    currentCurveParameter = trajectory.getClosestPoint(currentPose, splineSegmentGuessCurveParam, splineSegmentStartCurveParam, splineSegmentEndCurveParam);

    auto err = trajectory.error(Eigen::Vector2d(currentPose.position.x(), currentPose.position.y()), currentPose.getYaw(), currentCurveParameter, 0.);
    distanceError = err.first;
    lastAngleError = angleError;
    angleError = err.second;
}

FollowerStatus TrajectoryFollower::traverseTrajectory(Motion2D &motionCmd, const base::Pose &robotPose)
{
    motionCmd.translation = 0;
    motionCmd.rotation = 0;
    motionCmd.heading = 0;

    // Return if there is no trajectory to follow
    if(followerStatus != TRAJECTORY_FOLLOWING && followerStatus != EXEC_TURN_ON_SPOT && followerStatus != EXEC_LATERAL) {
        LOG_INFO_S << "Trajectory follower not active";
        return followerStatus;
    }

    lastPose = currentPose;
    currentPose = robotPose;

    computeErrors(robotPose);

    followerData.angleError = angleError;
    followerData.distanceError = distanceError;

    followerData.currentPose.position = currentPose.position;
    followerData.currentPose.orientation = currentPose.orientation;
    base::Pose2D refPose = trajectory.getIntermediatePoint(currentCurveParameter);
    followerData.splineReference.position = Eigen::Vector3d(refPose.position.x(), refPose.position.y(), 0.);
    followerData.splineReference.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(refPose.orientation, Eigen::Vector3d::UnitZ()));

    double distanceToEnd = trajectory.getDistToGoal(currentCurveParameter);
    lastPosError = posError;
    posError = (robotPose.position.head(2) - trajectory.getGoalPose().position).norm();

    bool reachedEnd = false;
    // If distance to trajectory finish set
    if (base::isUnset<double>(followerConf.trajectoryFinishDistance)) {
        // Only curve parameter
        if (!(currentCurveParameter < trajectory.getEndParam()))
            reachedEnd = true;
    } else {
        // Distance along curve to end point
        if (distanceToEnd <= followerConf.trajectoryFinishDistance)
        {
            if (followerConf.usePoseErrorReachedEndCheck)
                nearEnd = true;
            else
                reachedEnd = true;
        }

        if (posError <= followerConf.trajectoryFinishDistance)
            reachedEnd = true;
    }

    if (nearEnd) {
        if (posError > lastPosError)
            reachedEnd = true;
    }

    // If end reached
    if (reachedEnd) {
        // Trajectory finished
        nearEnd = false;
        LOG_INFO_S << "Trajectory follower finished";
        followerStatus = TRAJECTORY_FINISHED;
        return followerStatus;
    }

    if (checkTurnOnSpot()) {
        if ((angleError < -followerConf.pointTurnEnd
                || angleError > followerConf.pointTurnEnd)
                && std::signbit(lastAngleError) == std::signbit(angleError))
        {
            motionCmd.rotation = pointTurnDirection * followerConf.pointTurnVelocity;
            followerData.cmd = motionCmd.toBaseMotion2D();
            return followerStatus;
        }
        else
        {
            std::cout << "stopped Point-Turn. Switching to normal controller" << std::endl;
            pointTurn = false;
            pointTurnDirection = 1.;
            followerStatus = TRAJECTORY_FOLLOWING;
        }
    }

    // If trajectory follower running call the controller based on the
    // controller type
    if (controllerType == CONTROLLER_NO_ORIENTATION) {
        // No orientation controller update
        motionCmd = noOrientationController.update(trajectory.getSpeed(), distanceError, angleError);
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Chained controller update
        motionCmd = chainedController.update(trajectory.getSpeed(), distanceError, angleError, trajectory.getCurvature(currentCurveParameter),
                                             trajectory.getVariationOfCurvature(currentCurveParameter));
    } else if (controllerType == CONTROLLER_SAMSON) {
        motionCmd = samsonController.update(trajectory.getSpeed(), distanceError, angleError, trajectory.getCurvature(currentCurveParameter),
                                            trajectory.getVariationOfCurvature(currentCurveParameter));
    }

    if (trajectory.driveMode == ModeDiagonal)
    {
        motionCmd.rotation = angleLimit(trajectory.splineHeading(currentCurveParameter));
        motionCmd.heading = trajectory.splineHeading(currentCurveParameter) - refPose.orientation;
    }

    while (motionCmd.rotation > 2*M_PI || motionCmd.rotation < -2*M_PI)
        motionCmd.rotation += (motionCmd.rotation > 2*M_PI ? -1 : 1)*2*M_PI;

    // HACK: use damping factor to prevend oscillating steering behavior
    if (!base::isUnset<double>(followerConf.dampingAngleUpperLimit) && followerConf.dampingAngleUpperLimit > 0)
    {
        double dampingFactor = std::min(1., std::log(std::abs(base::Angle::fromRad(motionCmd.rotation).getDeg())+1.)*dampingCoefficient);
        motionCmd.rotation *= dampingFactor;
    }

    if(!base::isUnset< double >(followerConf.maxRotationalVelocity))
    {
        ///< Sets limits on rotational velocity
        motionCmd.rotation = std::min(motionCmd.rotation,  followerConf.maxRotationalVelocity);
        motionCmd.rotation = std::max(motionCmd.rotation, -followerConf.maxRotationalVelocity);
    }

    followerData.cmd = motionCmd.toBaseMotion2D();
    return followerStatus;
}

bool TrajectoryFollower::checkTurnOnSpot()
{
    if (pointTurn)
        return true;

    if(!(angleError > -followerConf.pointTurnStart && angleError < followerConf.pointTurnStart))
    {
        std::cout << "robot orientation : OUT OF BOUND ["  << angleError << ", " << followerConf.pointTurnStart << "]. starting point-turn" << std::endl;
        pointTurn = true;
        followerStatus = EXEC_TURN_ON_SPOT;
        this->trajectory.driveMode = ModeTurnOnTheSpot;

        if (angleError > 0)
            pointTurnDirection = -1.;

        lastAngleError = angleError;
        return true;
    }

    return false;
}