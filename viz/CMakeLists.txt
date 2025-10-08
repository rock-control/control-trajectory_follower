rock_find_qt4(OPTIONAL)
rock_find_qt5(OPTIONAL)

include(RockQt)

rock_qt_vizkit_plugin(
    TARGETPREFIX trajectory_follower-viz
    QT4_SUFFIX ""
    MISSINGQTDEPS_NOBUILD
    SOURCES SubTrajectoryVisualization.cpp
    DEPS trajectory_follower
    DEPS_PKGCONFIG osgViz
    MOC SubTrajectoryVisualization.hpp
    HEADERS SubTrajectoryVisualization.hpp)

