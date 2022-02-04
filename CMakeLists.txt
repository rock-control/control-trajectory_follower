# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.0)
project(trajectory_follower VERSION 0.1 DESCRIPTION "Trajectory Controller for Mobile Robots")
set(ROCK_USE_CXX11 TRUE)
find_package(Rock)
set(ROCK_TEST_ENABLED ON)

rock_init()
rock_standard_layout()
