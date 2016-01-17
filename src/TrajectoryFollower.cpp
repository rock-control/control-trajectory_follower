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
    isNewTrajectory = true;
}

void TrajectoryFollower::setNewTrajectory( const base::Trajectory &trajectory_,
        const base::Pose& robotPose )
{
    if(!configured)
        throw std::runtime_error("TrajectoryFollower not configured.");

    isNewTrajectory = true;

    // Sets the trajectory
    trajectory = trajectory_;

    // Sets the geometric resolution
    trajectory.spline.setGeometricResolution(trajectoryConfig.geometricResolution);

    // Curve parameter and length
    data.curveParameter = trajectory.spline.getStartParam();
    data.curveLength = trajectory.spline.getCurveLength(trajectoryConfig.geometricResolution);

    // Computes the current pose, reference pose and the errors
    data.currentPose = robotPose;
    computeErrors(robotPose);

    // Initialize based on controller type
    if(controllerType == CONTROLLER_NO_ORIENTATION) {
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
    base::Pose lastRobotPose = data.currentPose;

    // Transform robot pose into pose of the center of rotation
    data.currentPose.fromTransform(robotPose.toTransform() * poseTransform.toTransform());

    // Gets the heading of the current pose
    data.currentHeading = data.currentPose.getYaw();

    // Change heading based on direction of motion
    if(!trajectory.driveForward())
        data.currentHeading = angleLimit( data.currentHeading + M_PI );

    double direction = 1.0;
    if(isNewTrajectory) {
        Eigen::Vector3d error = trajectory.spline.poseError(data.currentPose.position,
                                data.currentHeading, data.curveParameter);

        data.distanceError  = error(0); // Distance error
        data.angleError     = error(1); // Heading error
        data.curveParameter = error(2); // Curve parameter of reference point

        // Setting reference values
        data.referenceHeading = trajectory.spline.getHeading( data.curveParameter );
        data.referencePose.position = trajectory.spline.getPoint( data.curveParameter );
        data.referencePose.orientation = AngleAxisd( data.referenceHeading, Vector3d::UnitZ() );
	
	isNewTrajectory = false;
	return;
    }

    movementVector = lastRobotPose.position.head(2) - data.currentPose.position.head(2);
    double movementDirection = atan2(movementVector.y(), movementVector.x());

    base::Angle diff(base::Angle::fromRad(movementDirection) - base::Angle::fromRad(data.currentPose.getYaw()));

    if(!trajectory.driveForward()) {
        if(fabs(diff.getRad()) > base::Angle::fromDeg(90).getRad())
            direction = -1;
    } else {
        if(fabs(diff.getRad()) < base::Angle::fromDeg(90).getRad())
            direction = -1;
    }

    // Find the closest point on the curve and gets the distance error and
    // heading error at this point
    double distanceMoved = (data.currentPose.position.head(2) - lastRobotPose.position.head(2)).norm() * direction;
    double errorMargin = distanceMoved * 0.1;
    errorMargin = std::max(errorMargin, trajectoryConfig.geometricResolution);
    double guess = trajectory.spline.advance(data.curveParameter, distanceMoved, trajectoryConfig.geometricResolution).first;

    //Find upper and lower bound for local search
    double start = trajectory.spline.advance(data.curveParameter, distanceMoved - errorMargin, trajectoryConfig.geometricResolution).first;
    double end = trajectory.spline.advance(data.curveParameter, distanceMoved + errorMargin, trajectoryConfig.geometricResolution).first;

    Eigen::Vector3d pos(data.currentPose.position);
    pos.z() = 0;

    double newParam = trajectory.spline.localClosestPointSearch(pos, guess, start, end, trajectoryConfig.geometricResolution);

    trajectorySegment = trajectory;
    trajectorySegment.spline.crop(start, end);

    data.distanceError  = trajectory.spline.distanceError(data.currentPose.position, newParam); // Distance error
    data.angleError     = trajectory.spline.headingError(data.currentHeading, newParam); // Heading error
    data.curveParameter = newParam; // Curve parameter of reference point

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

    // Computes reference pose and the errors
    computeErrors(robotPose);

    // Finding reached end condition
    double reachedEnd = false;

    data.distanceToEnd = fabs(trajectory.spline.getCurveLength(data.curveParameter, trajectoryConfig.geometricResolution));

    // If distance to trajectory finish set
    if (base::isUnset<double>(trajectoryConfig.trajectoryFinishDistance)) {
        // Only curve parameter
        if (!(data.curveParameter < trajectory.spline.getEndParam()))
            reachedEnd = true;
    } else {
        // Distance along curve to end point
        if (data.distanceToEnd <= trajectoryConfig.trajectoryFinishDistance)
            reachedEnd = true;
    }

    // If end reached
    if (reachedEnd) {
        // Trajectory finished
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

    return data.followerStatus;
}
