#include "TrajectoryTargetCalculator.hpp"
#include <base/Angle.hpp>
#include <base/logging.h>

using namespace Eigen;

trajectory_follower::TrajectoryTargetCalculator::TrajectoryTargetCalculator(double forwardLength) : forwardLength(forwardLength)
{
    newTrajectory = false;
    hasTrajectory = false;
    
    endReachedDistance = 0;
}

void trajectory_follower::TrajectoryTargetCalculator::setNewTrajectory(const base::Trajectory& trajectory)
{
    currentTrajectory = trajectory;
    currentTrajectory.spline.setGeometricResolution(0.001);
    
    endPoint.position = currentTrajectory.spline.getEndPoint();         
    endPoint.heading = currentTrajectory.spline.getHeading(currentTrajectory.spline.getEndParam());
    
    trajectoryLength = currentTrajectory.spline.length(currentTrajectory.spline.getStartParam(), currentTrajectory.spline.getEndParam(), 0.01);
    
    newTrajectory = true;
    hasTrajectory = true;
}

void trajectory_follower::TrajectoryTargetCalculator::removeTrajectory()
{
    hasTrajectory = false;
}

void trajectory_follower::TrajectoryTargetCalculator::setForwardLength(double length)
{
    forwardLength = length;
}

void trajectory_follower::TrajectoryTargetCalculator::setEndReachedDistance(double dist)
{
    endReachedDistance = dist;
}


double trajectory_follower::TrajectoryTargetCalculator::getDistanceXY(const base::Pose& robotPose, const base::Waypoint& wp) const
{
    return (Eigen::Vector2d(robotPose.position.x(), robotPose.position.y()) - Eigen::Vector2d(wp.position.x(), wp.position.y())).norm();
}


trajectory_follower::TrajectoryTargetCalculator::TARGET_CALCULATOR_STATUS trajectory_follower::TrajectoryTargetCalculator::traverseTrajectory(Eigen::Vector3d& targetPointb, const base::Pose& robotPose)
{
    if(!hasTrajectory)
        return REACHED_TRAJECTORY_END;

    base::Trajectory &trajectory(currentTrajectory);

    if(newTrajectory)
    {
        newTrajectory = false;
        para =  trajectory.spline.findOneClosestPoint(robotPose.position, 0.001);
    }    

    double distToEndXY = getDistanceXY(robotPose, endPoint);
    
    if ( para < trajectory.spline.getEndParam() )
    {
        double curParam = trajectory.spline.findOneClosestPoint(robotPose.position, para, 0.001);

        //TODO add proximity check
        //only traverse 'forward' on the trajectory
        if(curParam > para)
            para = curParam;
    }

    //check if we reached the end
    double drivenLength = trajectory.spline.length(trajectory.spline.getStartParam(), para, 0.01);
    
    if ( para >= trajectory.spline.getEndParam() ||
        (trajectoryLength - drivenLength < endReachedDistance && distToEndXY < endReachedDistance))
    {
        if(status != REACHED_TRAJECTORY_END)
        {
            LOG_INFO_S << "Reached end of trajectory" << std::endl;
            status = REACHED_TRAJECTORY_END;
            hasTrajectory = false;
        }
        return REACHED_TRAJECTORY_END;
    }

    std::pair<double, double> advancedPos = trajectory.spline.advance(para, forwardLength, 0.01);
    double targetPointParam = advancedPos.first;
    
    targetPoint.position = trajectory.spline.getPoint(targetPointParam);         
    targetPoint.heading = trajectory.spline.getHeading(targetPointParam);
    targetPointb = targetPoint.position;

    if(status != RUNNING)
    {
        LOG_INFO_S << "Started to follow trajectory" << std::endl;
        status = RUNNING;
    }

    return RUNNING;   
}
