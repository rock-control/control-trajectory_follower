#pragma once

#include "Controller.hpp"

namespace trajectory_follower
{

/** No orientation controller config */
struct NoOrientationControllerConfig
{
    double l1; ///< Position of reference point P(l1,0) on the robot chassis such that l1u1 > 0
    double K0; ///< Constant for the calculation of k(d, theta_e)
    bool useForwardAngleError, useForwardDistanceError;

    NoOrientationControllerConfig()
        : l1(base::unset< double >()),
          K0(base::unset< double >()),
          useForwardAngleError(false),
          useForwardDistanceError(false)
    {
    }
};
    
class NoOrientationController : public Controller {
public:
    NoOrientationController();

    NoOrientationController(const NoOrientationControllerConfig &config);

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature);
    virtual void reset() { };

private:
    double l1, K0;
};

}