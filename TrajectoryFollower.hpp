#ifndef TRAJECTORYFOLLOWER_HPP
#define TRAJECTORYFOLLOWER_HPP

#include <base/trajectory.h>
#include <base/motion_command.h>
#include <base/pose.h>

#include "TrajectoryControllerNoOrientation.hpp"
#include "TrajectoryControllerP.hpp"
#include "TrajectoryControllerPI.hpp"
#include "TrajectoryFollowerTypes.hpp"

namespace trajectory_follower {

class TrajectoryFollower
{

public:
    enum FOLLOWER_STATUS
    {
	RUNNING,
	REACHED_TRAJECTORY_END,
	INITIAL_STABILITY_FAILED,
    };

    TrajectoryFollower(double forwardLength, double gpsCenterofRotationOffset, int controllerType);
    
    /**
     * Sets a new trajectory
     * **/
    void setNewTrajectory(const base::Trajectory &trajectory);

    /**
     * Marks the current trajectory as traversed
     * */
    void removeTrajectory();
    
    /**
     * Gerenrates motion commands that should make the robot follow the
     * trajectory
     * */
    enum FOLLOWER_STATUS traverseTrajectory(Eigen::Vector2d &motionCmd, const base::Pose &robotPose);

    const TrajError &getControlError() const
    {
	return error;
    }

    const CurvePoint &getCurvePoint() const
    {
	return curvePoint;
    }
    
    const RobotPose & getPose() const
    {
	return pose;
    }
    
    trajectory_follower::noOrientation &getNoOrientationController()
    {
	return oTrajController_nO;
    }

    trajectory_follower::chainedProportional &getPController()
    {
	return oTrajController_P;
    }

    trajectory_follower::chainedProportionalIntegral &getPIController()
    {
	return oTrajController_PI;
    }

    /**
     * By default it is assumed that the x-axis of the robot's frame is pointing to the
     * front of the robot. So, if the y-axis should be the 'front-axis' PI / 2.0 has to be
     * passed here.
     */
    inline void setAddPoseErrorY(double rot_rad) {
        addPoseErrorY = rot_rad;
    }
    
private:
    bool bInitStable;
    bool newTrajectory;
    bool hasTrajectory;
    base::Trajectory currentTrajectory;

    FOLLOWER_STATUS status;
    
    double forwardLength;
    double gpsCenterofRotationOffset;
    int controllerType;
    
    double para;
    
    trajectory_follower::noOrientation oTrajController_nO;
    trajectory_follower::chainedProportional oTrajController_P;
    trajectory_follower::chainedProportionalIntegral oTrajController_PI;
    
    //only class members for debug reasons
    TrajError error;
    CurvePoint curvePoint;
    RobotPose pose;

    double addPoseErrorY;   

    TrajectoryFollower() {}
};

}
#endif // TRAJECTORYFOLLOWER_HPP
