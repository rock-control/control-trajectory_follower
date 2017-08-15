#include "TrajectoryFollower.hpp"
#include <base-logging/Logging.hpp>

#include <limits>

using namespace trajectory_follower;

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
      pointTurn(false),
      pointTurnDirection(1.),
      followerConf(followerConfig)
{
    followerStatus = TRAJECTORY_FINISHED;
    dampingCoefficient = base::unset< double >();

    // Configures the controller according to controller type
    switch (controllerType) {
    case CONTROLLER_NO_ORIENTATION:
        controller = new NoOrientationController(followerConf.noOrientationControllerConfig);
        break;
    case CONTROLLER_CHAINED:
        controller = new ChainedController(followerConf.chainedControllerConfig);
        break;
    case CONTROLLER_SAMSON:
        controller = new SamsonController(followerConf.samsonControllerConfig);
        break;
    default:
        throw std::runtime_error("Wrong or no controller type given.");
        break;
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

    controller->reset();

    // Sets the trajectory
    this->trajectory = trajectory;
    nearEnd = false;

    // Sets the geometric resolution
    this->trajectory.setGeometricResolution(followerConf.geometricResolution);

    // Curve parameter and length
    currentPose = robotPose;
    lastPose = currentPose;
    currentCurveParameter = this->trajectory.getStartParam();
    lastPosError = lastAngleError = distanceError = angleError = 0.;
    posError = lastPosError;

    followerData.currentPose.position = currentPose.position;
    followerData.currentPose.orientation = currentPose.orientation;
    base::Pose2D refPose = this->trajectory.getIntermediatePoint(currentCurveParameter);
    followerData.splineReference.position = Eigen::Vector3d(refPose.position.x(), refPose.position.y(), 0.);
    followerData.splineReference.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(refPose.orientation, Eigen::Vector3d::UnitZ()));
    followerData.splineSegmentStart = followerData.splineReference;
    followerData.splineSegmentEnd = followerData.splineReference;
    followerData.currentTrajectory.clear();
    followerData.currentTrajectory.push_back(this->trajectory.toBaseTrajectory());

    followerStatus = TRAJECTORY_FOLLOWING;
}

void TrajectoryFollower::computeErrors(const base::Pose& robotPose)
{
    lastPose = currentPose;
    currentPose = robotPose;

    // Gets the heading of the current pose
    double currentHeading = currentPose.getYaw();

    // Change heading based on direction of motion
    if(!trajectory.driveForward())
        currentHeading = SubTrajectory::angleLimit(currentHeading + M_PI);

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
    
    base::Pose2D splineStartPoint, splineEndPoint;
    splineStartPoint = trajectory.getIntermediatePoint(splineSegmentStartCurveParam);
    splineEndPoint = trajectory.getIntermediatePoint(splineSegmentEndCurveParam);
    followerData.splineSegmentStart.position.x() = splineStartPoint.position.x();
    followerData.splineSegmentStart.position.y() = splineStartPoint.position.y();
    followerData.splineSegmentEnd.position.x() = splineEndPoint.position.x();
    followerData.splineSegmentEnd.position.y() = splineEndPoint.position.y();
    followerData.splineSegmentStart.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(splineStartPoint.orientation, Eigen::Vector3d::UnitZ()));
    followerData.splineSegmentEnd.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(splineEndPoint.orientation, Eigen::Vector3d::UnitZ()));
    
    currentCurveParameter = trajectory.posSpline.localClosestPointSearch(currentPose.position, splineSegmentGuessCurveParam, splineSegmentStartCurveParam, splineSegmentEndCurveParam);

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
    if(followerStatus == TRAJECTORY_FINISHED) {
        LOG_INFO_S << "Trajectory follower not active";
        return followerStatus;
    }

    if (!base::isUnset<double>(followerConf.slamPoseErrorCheckEllipseX) && !base::isUnset<double>(followerConf.slamPoseErrorCheckEllipseY)) {
        double rx = std::min(followerConf.slamPoseErrorCheckEllipseX, 0.6), ry = std::min(followerConf.slamPoseErrorCheckEllipseY, 0.45);
        rx = std::max(rx, 0.01), ry = std::max(ry, 0.01);
        double angle = currentPose.getYaw()+angleError;

        const double slamPoseCheckVal = (Eigen::Rotation2Dd(angle) * (robotPose.position.head<2>() - currentPose.position.head<2>()))
                .cwiseQuotient(Eigen::Vector2d(rx, ry)).squaredNorm();


        if (!(slamPoseCheckVal <= 1.)) {
            if (followerStatus != SLAM_POSE_CHECK_FAILED) {
                std::cout << "SLAM_POSE_CHECK_FAILED! slamPoseCheckVal is " << slamPoseCheckVal << std::endl;
                lastFollowerStatus = followerStatus;
                followerStatus = SLAM_POSE_CHECK_FAILED;
            }

            return followerStatus;
        } else if (followerStatus == SLAM_POSE_CHECK_FAILED) {
            followerStatus = lastFollowerStatus;
        }
    }

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

    motionCmd = controller->update(trajectory.getSpeed(), distanceError, angleError, trajectory.getCurvature(currentCurveParameter),
                                   trajectory.getVariationOfCurvature(currentCurveParameter));

    // HACK: use damping factor to prevent oscillating steering behavior
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
