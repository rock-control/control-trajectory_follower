#ifndef TRAJECTORYFOLLOWER_HPP
#define TRAJECTORYFOLLOWER_HPP

#include "TrajectoryFollowerTypes.hpp"
#include "SubTrajectory.hpp"

namespace trajectory_follower
{

class Controller {
public:
    Controller()
        : configured(false)
    {
    }

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature) =0;
    virtual void reset() =0;

protected:
    bool configured;
    Motion2D motionCommand;
};

class NoOrientationController : public Controller {
public:
    NoOrientationController()
        : Controller()
    {
        l1 = base::unset<double>();
        K0 = base::unset<double>();
    }

    NoOrientationController(const NoOrientationControllerConfig &config)
        : NoOrientationController()
    {
        if (config.l1 <= 0)
        {
            throw std::runtime_error("l1 value must be greater than zero.");
        }

        if (config.K0 <= 0)
        {
            throw std::runtime_error("K0 value must be greater than zero.");
        }

        l1 = config.l1;
        K0 = config.K0;
        configured = true;
    }

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature);
    virtual void reset() { };

private:
    double l1, K0;
};

class ChainedController : public Controller {
public:
    ChainedController()
        : Controller()
    {
        K0 = base::unset<double>();
        K2 = base::unset<double>();
        K3 = base::unset<double>();
    }

    ChainedController(const ChainedControllerConfig &config)
        : ChainedController()
    {
        if (config.K2 <= 0 || config.K3 <= 0)
        {
            throw std::runtime_error("K2 & K3 value must be greater than zero.");
        }

        if (config.K0 <= 0 || base::isUnset< double >(config.K0))
        {
            std::cout << "ChainedController disabling integral" << std::endl;
            // Disabling integral
            K0 = 0;
        }
        else
        {
            K0 = config.K0;
        }

        K2 = config.K2;
        K3 = config.K3;
        configured = true;
    }

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature);
    virtual void reset() {
        controllerIntegral = 0.;
    };

private:
    double K0, K2, K3;
    double controllerIntegral;
};

class SamsonController : public Controller {
public:
    SamsonController()
        : Controller()
    {
        K2 = base::unset<double>();
        K3 = base::unset<double>();
    }

    SamsonController(const SamsonControllerConfig &config)
        : SamsonController()
    {
        if(config.K2 <= 0 || config.K3 <= 0)
        {
            throw std::runtime_error("K2 & K3 value must be greater than zero.");
        }
        
        K2 = config.K2;
        K3 = config.K3;
        configured = true;
    }

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature);
    virtual void reset() {  };

private:
    double K2, K3;
};

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
    TrajectoryFollower(const FollowerConfig& followerConfig);

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
    bool pointTurn;
    double pointTurnDirection;
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
    Controller *controller;
};

}

#endif // TRAJECTORYFOLLOWER_HPP