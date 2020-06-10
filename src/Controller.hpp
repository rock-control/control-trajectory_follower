#pragma once

#include "Motion2D.hpp"

namespace trajectory_follower
{

class Controller {
public:
    Controller()
        : configured(false)
    {
    }
    
    virtual ~Controller();

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature) =0;
    virtual void reset() =0;

protected:
    bool configured;
    Motion2D motionCommand;
};
}
