
PROJECT(trajectory_follower)

cmake_minimum_required(VERSION 2.6)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall)

find_package(PkgConfig)

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})

ADD_LIBRARY(trajectory_follower SHARED 
	    TrajectoryControllerNoOrientation.cpp 
	    RK4Integrator.cpp 
	    SimpleIntegrator.cpp 
	    TrajectoryControllerP.cpp
	    TrajectoryControllerPI.cpp)

TARGET_LINK_LIBRARIES(trajectory_follower 
                      ${BASE_LIBRARIES})
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS trajectory_follower
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES  
	TrajectoryControllerNoOrientation.hpp
	TrajectoryControllerP.hpp
	TrajectoryControllerPI.hpp
	RK4Integrator.hpp 
	SimpleIntegrator.hpp 
        TrajectoryControllerConfig.hpp
	DESTINATION include/${PROJECT_NAME}
        )

CONFIGURE_FILE(trajectory_follower.pc.in trajectory_follower.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/trajectory_follower.pc DESTINATION lib/pkgconfig)
