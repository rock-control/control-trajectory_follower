#include "TrajectoryFollower.hpp"
#include <base/Logging.hpp>

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
      controllerType(CONTROLLER_UNKNOWN )
{
    data.followerStatus = TRAJECTORY_FINISHED;
    nearEnd = false;
    data.splineReferenceErrorCoefficient = 0.;
}

TrajectoryFollower::TrajectoryFollower(const FollowerConfig& followerConfig)
    : configured(false),
      poseTransform(base::Pose( followerConfig.poseTransform)),
      trajectoryConfig(followerConfig.trajectoryConfig),
      controllerType(followerConfig.controllerType)
{
    data.followerStatus = TRAJECTORY_FINISHED;

    // Configures the controller according to controller type
    if(controllerType == CONTROLLER_NO_ORIENTATION) {
        // No orientation controller
        noOrientationController = NoOrientationController(followerConfig.noOrientationControllerConfig);
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Chained controller
        chainedController = ChainedController(followerConfig.chainedControllerConfig);
    } else {
        throw std::runtime_error("Wrong controller type given, it should be  "
                                 "either CONTROLLER_NO_ORIENTATION (0) or CONTROLLER_CHAINED (1).");
    }

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
    data.splineSegmentEndCurveParam = data.curveParameter;
    data.splineSegmentStartCurveParam = data.curveParameter;
    data.splineSegmentGuessCurveParam = data.curveParameter;
    data.distanceToEnd = data.curveLength;
    data.distanceMoved = 0;
    data.errorMargin = 0.;
    data.splineSegmentStartPose.position = Eigen::Vector3d(0., 0., 0.);
    data.splineSegmentStartPose.orientation = Eigen::Quaterniond::Identity();
    data.splineSegmentEndPose.position = Eigen::Vector3d(0., 0., 0.);
    data.splineSegmentEndPose.orientation = Eigen::Quaterniond::Identity();
    data.currentPose = robotPose;
    data.lastPose = data.currentPose;
    data.currentHeading = data.currentPose.getYaw();
    data.movementDirection.position = Eigen::Vector3d(0., 0., 0.);
    data.movementDirection.orientation = Eigen::Quaterniond::Identity();
    data.splineReferencePose.position = data.currentPose.position;
    data.splineReferencePose.orientation = data.currentPose.orientation;
    data.motionCommandViz.position = Eigen::Vector3d(0., 0., 0.);
    data.motionCommandViz.orientation = Eigen::Quaterniond::Identity();

    // Computes the current pose, reference pose and the errors
    computeErrors(robotPose);

    // Initialize based on controller type
    if (controllerType == CONTROLLER_NO_ORIENTATION) {
        // Resets the controlelr
        noOrientationController.reset();

        // Checks initial stability of the trajectory
        if (!noOrientationController.initialStable( data.distanceError,
                data.angleError, trajectory.spline.getCurvatureMax() )) {
            data.followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Reset the controller
        chainedController.reset();

        // Checks initial stability of the trajectory
        if (!chainedController.initialStable( data.distanceError,
                                              data.angleError,
                                              trajectory.spline.getCurvature(data.curveParameter),
                                              trajectory.spline.getCurvatureMax()) ) {
            data.followerStatus = INITIAL_STABILITY_FAILED;
            return;
        }
    }

    // Set state as following if stable
    data.followerStatus = TRAJECTORY_FOLLOWING;
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
    data.distanceMoved = movementVector.norm();
    data.movementDirection.position = data.currentPose.position;
    data.movementDirection.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(atan2(movementVector.y(), movementVector.x()), Eigen::Vector3d::UnitZ()));

    double direction = 1.;
    if (std::abs(data.movementDirection.getYaw() - data.currentHeading) > base::Angle::fromDeg(90).getRad())
        direction = -1.;

    data.errorMargin = data.distanceMoved*data.splineReferenceErrorCoefficient;
    if (!base::isUnset<double>(trajectoryConfig.splineReferenceError))
        data.errorMargin += std::abs(trajectoryConfig.splineReferenceError);

    //Find upper and lower bound for local search
    double forwardLength, backwardLength;
    forwardLength = data.distanceMoved + data.errorMargin;
    backwardLength = forwardLength;

    if (!base::isUnset<double>(trajectoryConfig.maxForwardLenght))
        forwardLength = std::min(forwardLength, trajectoryConfig.maxForwardLenght);

    if (!base::isUnset<double>(trajectoryConfig.maxBackwardLenght))
        backwardLength = std::min(backwardLength, trajectoryConfig.maxBackwardLenght);

    if (trajectory.spline.length(trajectory.spline.getStartParam(), data.curveParameter, trajectoryConfig.geometricResolution) > backwardLength)
        data.splineSegmentStartCurveParam = trajectory.spline.advance(data.curveParameter, -backwardLength, trajectoryConfig.geometricResolution).first;
    else
        data.splineSegmentStartCurveParam = trajectory.spline.getStartParam();

    if (data.distanceToEnd > forwardLength)
        data.splineSegmentEndCurveParam = trajectory.spline.advance(data.curveParameter, forwardLength, trajectoryConfig.geometricResolution).first;
    else
        data.splineSegmentEndCurveParam = trajectory.spline.getEndParam();

    double dist = data.distanceMoved*direction;
    if ((dist > 0. && data.distanceToEnd > dist) || (dist < 0. && (data.curveLength-data.distanceToEnd) > std::abs(dist)))
        data.splineSegmentGuessCurveParam = trajectory.spline.advance(data.curveParameter, dist, trajectoryConfig.geometricResolution).first;
    else if (dist > 0.)
        data.splineSegmentGuessCurveParam = trajectory.spline.getEndParam();
    else
        data.splineSegmentGuessCurveParam = trajectory.spline.getStartParam();

    Eigen::Vector3d curPos(data.currentPose.position);
    curPos.z() = 0;

    data.curveParameter = trajectory.spline.localClosestPointSearch(curPos, data.splineSegmentGuessCurveParam, data.splineSegmentStartCurveParam, data.splineSegmentEndCurveParam, trajectoryConfig.geometricResolution);
    data.splineSegmentStartPose.position = trajectory.spline.getPoint(data.splineSegmentStartCurveParam);
    data.splineSegmentStartPose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(trajectory.spline.getHeading(data.splineSegmentStartCurveParam), Eigen::Vector3d::UnitZ()));

    if (data.splineSegmentEndCurveParam <= trajectory.spline.getEndParam()) {
        data.splineSegmentEndPose.position = trajectory.spline.getPoint(data.splineSegmentEndCurveParam);
        data.splineSegmentEndPose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(trajectory.spline.getHeading(data.splineSegmentEndCurveParam), Eigen::Vector3d::UnitZ()));
    }

    double targetCurveParam = data.curveParameter;
    if (controllerType == trajectory_follower::CONTROLLER_NO_ORIENTATION) {
        if (!base::isUnset<double>(noOrientationController.getConfig().l1))
            targetCurveParam = trajectory.spline.advance(data.curveParameter, noOrientationController.getConfig().l1, trajectoryConfig.geometricResolution).first;
    }

    data.distanceError  = trajectory.spline.distanceError(data.currentPose.position, targetCurveParam); // Distance error
    data.angleError     = angleLimit(trajectory.spline.headingError(data.currentHeading, targetCurveParam)); // Heading error

    // Setting reference values
    data.referenceHeading = trajectory.spline.getHeading(data.curveParameter);
    data.referencePose.position = trajectory.spline.getPoint(data.curveParameter);
    data.referencePose.orientation = AngleAxisd(data.referenceHeading, Vector3d::UnitZ());
}

FollowerStatus TrajectoryFollower::traverseTrajectory(
    base::commands::Motion2D &motionCmd,
    const base::Pose &robotPose )
{
    motionCmd.translation = 0;
    motionCmd.rotation = 0;

    // Return if there is no trajectory to follow
    if(data.followerStatus != TRAJECTORY_FOLLOWING) {
        LOG_INFO_S << "Trajectory follower not active";
        return data.followerStatus;
    }

    data.time = base::Time::now();

    data.splineReferencePose.position = data.referencePose.position;
    data.splineReferencePose.orientation = data.referencePose.orientation;

    // Computes reference pose and the errors
    computeErrors(robotPose);

    // Finding reached end condition
    bool reachedEnd = false;

    data.distanceToEnd = trajectory.spline.getCurveLength(data.curveParameter, trajectoryConfig.geometricResolution);
    data.posError = (robotPose.position.head(2) - data.goalPose.position.head(2)).norm();
    data.lastPosError = (data.lastPose.position.head(2) - data.goalPose.position.head(2)).norm();

    // If distance to trajectory finish set
    if (base::isUnset<double>(trajectoryConfig.trajectoryFinishDistance)) {
        // Only curve parameter
        if (!(data.curveParameter < trajectory.spline.getEndParam()))
            reachedEnd = true;
    } else {
        // Distance along curve to end point
        if (data.distanceToEnd <= trajectoryConfig.trajectoryFinishDistance)
            nearEnd = true;

        if (data.posError <= trajectoryConfig.trajectoryFinishDistance)
            reachedEnd = true;
    }

    if (nearEnd) {
        if (static_cast<int>(data.posError*100.) > static_cast<int>(data.lastPosError*100.))
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

    // If trajectory follower running call the controller based on the
    // controller type
    if (controllerType == CONTROLLER_NO_ORIENTATION) {
        // No orientation controller update
        motionCmd = noOrientationController.update(trajectory.speed,
                    data.distanceError, data.angleError);
    } else if (controllerType == CONTROLLER_CHAINED) {
        // Chained controller update
        motionCmd = chainedController.update( trajectory.speed,
                                              data.distanceError,
                                              data.angleError,
                                              trajectory.spline.getCurvature(data.curveParameter),
                                              trajectory.spline.getVariationOfCurvature(data.curveParameter));
    }

    while (motionCmd.rotation > 2*M_PI || motionCmd.rotation < -2*M_PI)
        motionCmd.rotation += (motionCmd.rotation > 2*M_PI ? -1 : 1)*2*M_PI;

    data.motionCommandViz.position = robotPose.position;
    data.motionCommandViz.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(motionCmd.rotation, Eigen::Vector3d::UnitZ()));

    return data.followerStatus;
}
