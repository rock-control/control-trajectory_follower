#ifndef TRAJECTORYTARGETCALCULATOR_H
#define TRAJECTORYTARGETCALCULATOR_H

#include <base/trajectory.h>
#include <base/motion_command.h>
#include <base/pose.h>
#include <base/waypoint.h>

namespace trajectory_follower {

class TrajectoryTargetCalculator
{
public:
    enum TARGET_CALCULATOR_STATUS
    {
        RUNNING,
        REACHED_TRAJECTORY_END,
    };

    TrajectoryTargetCalculator(double forwardLength);
    
    /**
     * Sets a new trajectory
     * **/
    void setNewTrajectory(const base::Trajectory &trajectory);

    /**
     * Marks the current trajectory as traversed
     * */
    void removeTrajectory();
    
    /**
     * Generates a new target point on the trajectory
     * */
    enum TARGET_CALCULATOR_STATUS traverseTrajectory(Eigen::Vector3d &targetPoint, const base::Pose &robotPose);

    void setForwardLength(double length);
    
    /**
     * If the distance between the end point of the spline 
     * and the robot is below this distance, the trajectory
     * is considered driven;
     * */
    void setEndReachedDistance(double dist);
    
    const base::Waypoint &getTargetPoint() const
    {
        return targetPoint;
    }
    
private:
    double getDistanceXY(const base::Pose &robotPose, const base::Waypoint &wp) const;
    
    bool newTrajectory;
    bool hasTrajectory;
    base::Trajectory currentTrajectory;

    TARGET_CALCULATOR_STATUS status;
    
    double endReachedDistance;
    double trajectoryLength;
    
    double forwardLength;
    double para;
    
    base::Waypoint targetPoint;
    base::Waypoint endPoint;

    double addPoseErrorY;   
};

}
#endif // TRAJECTORYTARGETCALCULATOR_H
