rock_library(trajectory_follower
    SOURCES TrajectoryControllerNoOrientation.cpp 
            TrajectoryTargetCalculator.cpp
	    RK4Integrator.cpp  
	    TrajectoryFollower.cpp
	    SimpleIntegrator.cpp 
	    TrajectoryControllerP.cpp
	    TrajectoryControllerPI.cpp
    HEADERS TrajectoryFollower.hpp
	TrajectoryControllerNoOrientation.hpp
	TrajectoryControllerP.hpp
	TrajectoryControllerPI.hpp
	RK4Integrator.hpp 
	SimpleIntegrator.hpp 
        TrajectoryControllerConfig.hpp
        TrajectoryFollowerTypes.hpp
        TrajectoryTargetCalculator.hpp
    DEPS_PKGCONFIG eigen3 base-types )
