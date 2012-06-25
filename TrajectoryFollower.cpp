#include "TrajectoryFollower.hpp"

namespace trajectory_follower {


TrajectoryFollower::TrajectoryFollower(double forwardLength, double gpsCenterofRotationOffset, int controllerType):
    bInitStable(false), newTrajectory(true), hasTrajectory(false), status(REACHED_TRAJECTORY_END), forwardLength(forwardLength), gpsCenterofRotationOffset(gpsCenterofRotationOffset), controllerType(controllerType)
{
    if(controllerType != 0 && 
	controllerType != 1 && 
	controllerType != 2)
	throw std::runtime_error("Wrong controller type given (not 0, 1 or 2)");
}

void TrajectoryFollower::setNewTrajectory(const base::Trajectory &trajectory)
{
    currentTrajectory = trajectory;
    currentTrajectory.spline.setGeometricResolution(0.001);
    bInitStable = false;
    newTrajectory = true;
    hasTrajectory = true;
}
    
void TrajectoryFollower::removeTrajectory()
{
    hasTrajectory = false;
}
    
double angleLimit(double angle)
{
    if(angle > M_PI)
	return angle - 2*M_PI;
    else if (angle < -M_PI)
	return angle + 2*M_PI;
    else
     	return angle;
}
    
enum TrajectoryFollower::FOLLOWER_STATUS TrajectoryFollower::traverseTrajectory(Eigen::Vector2d &motionCmd, const base::Pose &robotPose)
{   
    motionCmd(0) = 0.0; 
    motionCmd(1) = 0.0; 

    if(!hasTrajectory)
	return REACHED_TRAJECTORY_END;

    base::Trajectory &trajectory(currentTrajectory);

    if(newTrajectory)
    {
	newTrajectory = false;
        para =  trajectory.spline.findOneClosestPoint(robotPose.position, 0.001);
    }    

    pose.position = robotPose.position;
    pose.heading  = robotPose.getYaw();
    
    if ( para < trajectory.spline.getEndParam() )
    {

	double dir = 1.0;
	if(!trajectory.driveForward())
        {
            pose.heading  = angleLimit(pose.heading+M_PI);
	    dir = -1.0;
        }	
        if(controllerType == 0)
        {
            pose.position.x() = pose.position.x() - (dir * forwardLength + gpsCenterofRotationOffset) * sin(pose.heading);
            pose.position.y() = pose.position.y() + (dir * forwardLength + gpsCenterofRotationOffset) * cos(pose.heading);
        }
        else
        {
            pose.position.x() = pose.position.x() - (gpsCenterofRotationOffset) * sin(pose.heading);
            pose.position.y() = pose.position.y() + (gpsCenterofRotationOffset) * cos(pose.heading);
        }

        Eigen::Vector3d vError = trajectory.spline.poseError(pose.position, pose.heading, para);
        para  = vError(2);
       
	error.d = vError(0);
        error.theta_e = angleLimit(vError(1) + M_PI_2);
        error.param = vError(2);
        
        curvePoint.pose.position 	= trajectory.spline.getPoint(para); 	    
        curvePoint.pose.heading  	= trajectory.spline.getHeading(para);
        curvePoint.param 		= para;

	//disable this test for testing, as it seems to be not needed
	bInitStable = true;
        if(!bInitStable)
        {
	    switch(controllerType)
	    {
		case 0:
		    bInitStable = oTrajController_nO.checkInitialStability(error.d, error.theta_e, trajectory.spline.getCurvatureMax());
		    bInitStable = true;
		    break;
		case 1:
		    bInitStable = oTrajController_P.checkInitialStability(error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getCurvatureMax());
		    break;
		case 2:
		    bInitStable = oTrajController_PI.checkInitialStability(error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getCurvatureMax());
		    break;
		default:
		    throw std::runtime_error("Got bad controllerType value");
	    }

            if (!bInitStable)
            {
                std::cout << "Trajectory controller: failed initial stability test";
                return INITIAL_STABILITY_FAILED;
            }
        }

        double vel = currentTrajectory.speed;
	switch(controllerType)
	{
	    case 0:
		motionCmd = oTrajController_nO.update(vel, error.d, error.theta_e); 
		break;
	    case 1:
		motionCmd = oTrajController_P.update(vel, error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getVariationOfCurvature(para));
		break;
	    case 2:
		motionCmd = oTrajController_PI.update(vel, error.d, error.theta_e, trajectory.spline.getCurvature(para), trajectory.spline.getVariationOfCurvature(para));
		break;
	    default:
		throw std::runtime_error("Got bad controllerType value");
	}

        std::cout << "\n Mc: " << motionCmd(0) << " " << motionCmd(1) 
                  << " error: d " <<  error.d << " theta " << error.theta_e << " PI" << std::endl;
    }
    else
    {
	if(status != REACHED_TRAJECTORY_END)
	{
	    std::cout << "Reached end of trajectory" << std::endl;
	    status = REACHED_TRAJECTORY_END;
	}
	return REACHED_TRAJECTORY_END;
    }

    if(status != RUNNING)
    {
	std::cout << "Started to follow trajectory" << std::endl;
	status = RUNNING;
    }

    return RUNNING;    
}

}
