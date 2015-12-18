#ifndef TRAJECTORYFOLLOWER_HPP
#define TRAJECTORYFOLLOWER_HPP

#include "TrajectoryFollowerTypes.hpp"
#include "NoOrientationController.hpp"
#include "ChainedController.hpp"

namespace trajectory_follower 
{

    class TrajectoryFollower
    {
        public:
            TrajectoryFollower();
            TrajectoryFollower( const FollowerConfig& followerConfig );

            /**
             * Sets a new trajectory
             */
            void setNewTrajectory( const base::Trajectory &trajectory_,
                    const base::Pose& robotPose );

            /**
             * Marks the current trajectory as traversed
             */
            inline void removeTrajectory() 
            { data.followerStatus = TRAJECTORY_FINISHED; }

            /**
             * Gerenrates motion commands that should make the robot follow the
             * trajectory
             */
            FollowerStatus traverseTrajectory( base::commands::Motion2D &motionCmd, 
                    const base::Pose &robotPose );

            void computeErrors( const base::Pose& robotPose );

            double angleLimit( double angle );

            const FollowerData& getData() { return data; }
        private:
            bool configured;

            base::Trajectory trajectory;
            base::Vector3d trajectoryEndPoint;

            base::Pose poseTransform;
            TrajectoryConfig trajectoryConfig;
            ControllerType controllerType;

            NoOrientationController noOrientationController;
            ChainedController chainedController;

            FollowerData data;
    };
}
#endif // TRAJECTORYFOLLOWER_HPP
