#ifndef TRAJECTORYFOLLOWER_HPP
#define TRAJECTORYFOLLOWER_HPP

#include "TrajectoryFollowerTypes.hpp"
#include "SubTrajectory.hpp"

#include "Controller.hpp"
#include <memory>

namespace trajectory_follower
{

/**
 * TrajectoryFollower class combines reference pose finder and
 * trajectory controller
 **/
class TrajectoryFollower
{
public:
    /** Default constructor */
    TrajectoryFollower();

    /** Constructor which takes in config.
     *
     * Before using the follower make sure that the object is created
     * using the correct config and this constructor, otherwise the
     * controller will cause runtime error */
    TrajectoryFollower(const FollowerConfig& followerConfig);

    /** Sets a new trajectory
     *
     * Here it checks for the initial stability of the trajectory
     */
    void setNewTrajectory(const SubTrajectory &trajectory, const base::Pose& robotPose);

    /**
     * @brief Here we set the heading error tolerance for the orientation alignment for planner defined point-turns
     * 
     */
    void setHeadingErrorTolerance(const double tolerance);

    /**
     * Marks the current trajectory as traversed
     *
     * Stops the current trajectory following and removes the trajectory
     */
    void removeTrajectory()
    {
        followerStatus = TRAJECTORY_FINISHED;
    }

    /**
     * Generates motion commands that should make the robot follow the
     * trajectory
     */
    FollowerStatus traverseTrajectory(Motion2D &motionCmd, const base::Pose &robotPose);

    /** Computes the reference pose and the error relative to this pose */
    void computeErrors(const base::Pose& robotPose);

    /** Returns the current follower data */
    const FollowerData& getData() {
        return followerData;
    }

    bool checkTurnOnSpot();

private:
    bool configured;
    ControllerType controllerType;
    bool automaticPointTurn;
    double pointTurnDirection;
    double headingErrorTolerance;
    bool nearEnd;
    double dampingCoefficient;
    base::Pose currentPose;
    base::Pose lastPose;
    double lastPosError;
    double currentCurveParameter;
    double distanceError;
    double angleError, lastAngleError;
    double posError;
    double splineReferenceErrorCoefficient;
    FollowerData followerData;
    FollowerStatus followerStatus, lastFollowerStatus;
    SubTrajectory trajectory;
    FollowerConfig followerConf;
    std::unique_ptr<Controller> controller;
};

}

#endif // TRAJECTORYFOLLOWER_HPP
