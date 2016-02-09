rock_library(trajectory_follower
    SOURCES NoOrientationController.cpp 
            ChainedController.cpp
	    TrajectoryFollower.cpp
    HEADERS TrajectoryFollower.hpp
	NoOrientationController.hpp
        ChainedController.hpp
        TrajectoryFollowerTypes.hpp
    DEPS_PKGCONFIG eigen3 base-types )