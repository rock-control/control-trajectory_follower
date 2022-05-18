rock_find_qt5(Core)
rock_vizkit_plugin(trajectory_follower-viz
    SubTrajectoryVisualization.cpp
    DEPS trajectory_follower
    DEPS_PKGCONFIG osgViz
    MOC SubTrajectoryVisualization.hpp
    HEADERS SubTrajectoryVisualization.hpp)

