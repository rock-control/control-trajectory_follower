rock_library(trajectory_follower
    SOURCES NoOrientationController.cpp 
            ChainedController.cpp
	    TrajectoryFollower.cpp
	    SamsonController.cpp
    HEADERS TrajectoryFollower.hpp
	NoOrientationController.hpp
        ChainedController.hpp
        TrajectoryFollowerTypes.hpp
        SamsonController.hpp
    DEPS_PKGCONFIG eigen3 base-types )