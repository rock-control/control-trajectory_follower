#ifndef __TRAJECTORYCONTROLLER_CONFIG_H_
#define __TRAJECTORYCONTROLLER_CONFIG_H_

namespace trajectory_follower {

struct ControllerConfig
{
     /* Define the chainedProportionalIntegral controller to be used. Values: 0: No orientation, 1: chainedProportional, 2: chainedProportionalIntegral */
    int controllerType; 
    /** 
    * Forward velocity of the robot
    * in m/s
    */
    double forwardVelocity;

    /* moving the pose estimation point forward for no orientation controller"*/
    double forwardLength;

    /** Offset between GPS (w.r.t which Pose Estimator works) and approximated center of rotation
    **/
    double gpsCenterOfRotationOffset;

    /** K0 gain for no orientation controller*/
    double K0_noOrientation;

    /** K2 gain for P controller*/
    double K2_P;

    /** K3 gain for P controller*/
    double K3_P;

    /** K0 gain for PI_controller*/
    double K0_PI;
    
    /** K2 gain for PI controller*/
    double K2_PI;

    /** K3 gain for PI controller*/
    double K3_PI;
};

}
#endif //__TRAJECTORYCONTROLLER_CONFIG_H_
