rock_library(trajectory_follower
    SOURCES TrajectoryFollower.cpp
            SubTrajectory.cpp
    HEADERS TrajectoryFollower.hpp
            TrajectoryFollowerTypes.hpp
            SubTrajectory.hpp
            Motion2D.hpp
    DEPS_PKGCONFIG eigen3 base-types )