#pragma once
#include "Controller.hpp"

namespace trajectory_follower
{
    
/** Chained controller config */
struct ChainedControllerConfig
{
    double K0; ///< Integrator constant
    double K2; ///< Controller constant
    double K3; ///< Controller constant

    ChainedControllerConfig()
        : K0(base::unset< double >()),
          K2(base::unset< double >()),
          K3(base::unset< double >())
    {
    }
};

    
class ChainedController : public Controller {
public:
    ChainedController();
    ChainedController(const ChainedControllerConfig &config);

    virtual Motion2D& update(double speed, double distanceError, double angleError, double curvature, double variationOfCurvature);
    virtual void reset() {
        controllerIntegral = 0.;
    };

private:
    double K0, K2, K3;
    double controllerIntegral;
};
}
