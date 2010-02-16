
PROJECT(trajectory_follower)

cmake_minimum_required(VERSION 2.6)

#add definitions, compiler switches, etc.
ADD_DEFINITIONS(-Wall)

#list all source files here
#ADD_EXECUTABLE(driver main.cpp interface.cpp)
find_package(PkgConfig)

pkg_check_modules(BASE_TYPES REQUIRED "base-types")
INCLUDE_DIRECTORIES(${BASE_TYPES_INCLUDE_DIRS})
LINK_DIRECTORIES(${BASE_TYPES_LIBRARY_DIRS})

ADD_LIBRARY(trajectory_follower SHARED 
	    TrajectoryController_NoOrientation.cpp 
	    rk4_integrator.cpp 
	    simpleintegrator.cpp 
	    TrajectoryController_P.cpp
	    TrajectoryController_PI.cpp)

TARGET_LINK_LIBRARIES(trajectory_follower 
                      ${BASE_LIBRARIES})
 
CONFIGURE_FILE(Doxyfile.in Doxyfile @ONLY)
ADD_CUSTOM_TARGET(doc doxygen Doxyfile)
INSTALL(TARGETS trajectory_follower
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)
INSTALL(FILES  
	TrajectoryController_NoOrientation.h 
	TrajectoryController_P.h
	TrajectoryController_PI.h
	rk4_integrator.hpp 
	simpleintegrator.hpp 
	DESTINATION include)

CONFIGURE_FILE(trajectory_follower.pc.in trajectory_follower.pc @ONLY)
INSTALL(FILES ${CMAKE_BINARY_DIR}/trajectory_follower.pc DESTINATION lib/pkgconfig)
