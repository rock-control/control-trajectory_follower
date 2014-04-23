#include "TrajectoryTargetCalculator.hpp"
#include <base/Angle.hpp>
#include <base/logging.h>

using namespace Eigen;

trajectory_follower::TrajectoryTargetCalculator::TrajectoryTargetCalculator(double forwardLength) : forwardLength(forwardLength)
{
    newTrajectory = false;
    hasTrajectory = false;
}

void trajectory_follower::TrajectoryTargetCalculator::setNewTrajectory(const base::Trajectory& trajectory)
{
    currentTrajectory = trajectory;
    currentTrajectory.spline.setGeometricResolution(0.001);
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

    if ( para < trajectory.spline.getEndParam() )
    {
        double curParam = trajectory.spline.findOneClosestPoint(robotPose.position, para, 0.001);

        //TODO add proximity check
        //only traverse 'forward' on the trajectory
        if(curParam > para)
            para = curParam;
            
        std::pair<double, double> advancedPos = trajectory.spline.advance(para, forwardLength, 0.01);
        double targetPointParam = advancedPos.first;
        
        targetPoint.position = trajectory.spline.getPoint(targetPointParam);         
        targetPoint.heading = trajectory.spline.getHeading(targetPointParam);
        targetPointb = targetPoint.position;
    }
    else
    {
        if(status != REACHED_TRAJECTORY_END)
        {
            LOG_INFO_S << "Reached end of trajectory" << std::endl;
            status = REACHED_TRAJECTORY_END;
        }
        return REACHED_TRAJECTORY_END;
    }

    if(status != RUNNING)
    {
        LOG_INFO_S << "Started to follow trajectory" << std::endl;
        status = RUNNING;
    }

    return RUNNING;   
}
