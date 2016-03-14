#ifndef TRAJECTORYFOLLOWER_HPP
#define TRAJECTORYFOLLOWER_HPP

#include "TrajectoryFollowerTypes.hpp"
#include "NoOrientationController.hpp"
#include "ChainedController.hpp"
#include "SamsonController.hpp"
#include "SubTrajectory.hpp"

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

    /** Contructor which takes in config.
     *
     * Before using the follower make sure that the object is created
     * using the correct config and this contructor, otherwise the
     * controller will cause runtime error */
    TrajectoryFollower( const FollowerConfig& followerConfig );

    /** Sets a new trajectory
     *
     * Here it checks for the initial stability of the trajectory
     */
    void setNewTrajectory(const SubTrajectory &trajectory, const base::Pose& robotPose);

    /**
     * Marks the current trajectory as traversed
     *
     * Stops the current trajectory following and removes the trajectory
     */
    inline void removeTrajectory()
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

    /** Converts all values to within +/- M_PI */
    double angleLimit(double angle);

    /** Returns the current follower data */
    const FollowerData& getData() {
        return followerData;
    }
    
    bool checkTurnOnSpot();

private:
    bool configured; ///< True if configured properly
    bool nearEnd;
    double dampingCoefficient;
    bool pointTurn;
    double pointTurnDirection;
    base::Pose currentPose;
    base::Pose lastPose;
    double lastPosError;
    double currentCurveParameter;
    double distanceError;
    double angleError, lastAngleError;
    double posError;
    double splineReferenceErrorCoefficient;
    FollowerData followerData;
    FollowerStatus followerStatus;

    SubTrajectory trajectory; ///< Active trajectory
    ControllerType controllerType; ///< Controller type

    NoOrientationController noOrientationController; ///< No orientation controller
    ChainedController chainedController; ///< Chained controller
    SamsonController samsonController;

    FollowerConfig followerConf;
};

}

#endif // TRAJECTORYFOLLOWER_HPP