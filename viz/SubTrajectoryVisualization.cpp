#include <iostream>
#include "SubTrajectoryVisualization.hpp"

using namespace vizkit3d;

struct SubTrajectoryVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    std::vector<trajectory_follower::SubTrajectory> data;
};


SubTrajectoryVisualization::SubTrajectoryVisualization()
    : p(new Data)
{
}

SubTrajectoryVisualization::~SubTrajectoryVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> SubTrajectoryVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void SubTrajectoryVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void SubTrajectoryVisualization::updateDataIntern(std::vector<trajectory_follower::SubTrajectory> const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SubTrajectoryVisualization)

