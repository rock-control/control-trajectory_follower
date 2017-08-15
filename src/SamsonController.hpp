#pragma once

#include "Controller.hpp"

namespace trajectory_follower
{
    
struct SamsonControllerConfig
{
    double K2;
    double K3;

    SamsonControllerConfig()
        : K2(base::unset< double >()),
          K3(base::unset< double >())
    {
    }
};

class SamsonController : public Controller {
public:
    SamsonController();

    SamsonController(const SamsonControllerConfig &config);

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature);
    virtual void reset() {  };

private:
    double K2, K3;
};
}
