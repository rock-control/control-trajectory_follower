#ifndef TRAJECTORYFOLLOWER_HPP
#define TRAJECTORYFOLLOWER_HPP

#include "TrajectoryFollowerTypes.hpp"
#include "NoOrientationController.hpp"
#include "ChainedController.hpp"

namespace trajectory_follower 
{

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
            TrajectoryFollower( const FollowerConfig& followerConfig );

            /** Sets a new trajectory
             *
             * Here it checks for the initial stability of the trajectory 
             */
            void setNewTrajectory( const base::Trajectory &trajectory_,
                    const base::samples::RigidBodyState& robotPose );

            /**
             * Marks the current trajectory as traversed
             *
             * Stops the current trajectory following and removes the trajectory
             */
            inline void removeTrajectory() 
            { data.followerStatus = TRAJECTORY_FINISHED; }

            /**
             * Generates motion commands that should make the robot follow the
             * trajectory
             */
            FollowerStatus traverseTrajectory( base::commands::Motion2D &motionCmd, 
                    const base::samples::RigidBodyState &robotPose );

            /** Computes the reference pose and the error relative to this pose */
            void computeErrors( const base::samples::RigidBodyState& robotPose );

            /** Converts all values to within +/- M_PI */
            double angleLimit( double angle );

            /** Returns the current follower data */
            const FollowerData& getData() { return data; }

        private:
            bool configured; ///< True if configured properly

            base::Trajectory trajectory; ///< Active trajectory 
            double direction; ///< 1 for forward, -1 for reverse

            base::samples::RigidBodyState poseTransform; ///< Transforms robot pose to center of rotation pose
            base::samples::RigidBodyState backTransform; ///< Transforms robot pose to backward rotation

            TrajectoryConfig trajectoryConfig; ///< Config for trajectory
            ControllerType controllerType; ///< Controller type

            NoOrientationController noOrientationController; ///< No orientation controller
            ChainedController chainedController; ///< Chained controller

            FollowerData data; ///< Follower data
    };
}
#endif // TRAJECTORYFOLLOWER_HPP
