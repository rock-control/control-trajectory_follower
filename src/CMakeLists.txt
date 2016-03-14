rock_library(trajectory_follower
    SOURCES NoOrientationController.cpp 
            ChainedController.cpp
            TrajectoryFollower.cpp
            SamsonController.cpp
            SubTrajectory.cpp
    HEADERS TrajectoryFollower.hpp
            NoOrientationController.hpp
            ChainedController.hpp
            TrajectoryFollowerTypes.hpp
            SamsonController.hpp
            SubTrajectory.hpp
            Motion2D.hpp
    DEPS_PKGCONFIG eigen3 base-types )