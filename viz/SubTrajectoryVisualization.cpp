#include <iostream>
#include "SubTrajectoryVisualization.hpp"
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/OsgViz.hpp>

using namespace vizkit3d;

struct SubTrajectoryVisualization::Data {
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
    return new osg::PositionAttitudeTransform();
}

void SubTrajectoryVisualization::updateMainNode ( osg::Node* node )
{
    osg::PositionAttitudeTransform* geode = static_cast<osg::PositionAttitudeTransform*>(node);
    geode->removeChildren(0, geode->getNumChildren());

    osgviz::PrimitivesFactory* fac = osgviz::OsgViz::getInstance()->getModuleInstance<osgviz::PrimitivesFactory>("PrimitivesFactory");
    std::vector<osg::Vec3> osgPoints;
    
    for(const trajectory_follower::SubTrajectory& traj : p->data)
    {
        const base::geometry::Spline<3>& spline = traj.posSpline;
        double stepSize = (spline.getEndParam() - spline.getStartParam()) / (spline.getCurveLength() / 0.05);
        for(double param = spline.getStartParam(); param <= spline.getEndParam(); param += stepSize )
        {
            const Eigen::Vector3d splinePoint = spline.getPoint(param);
            osgPoints.emplace_back(splinePoint.x(), splinePoint.y(), splinePoint.z());
        }
    }
    
    const osg::Vec4 color(1, 1, 0, 1);
    auto prim = fac->createLinesNode(color, osgPoints);
    geode->addChild(prim);
}

void SubTrajectoryVisualization::updateDataIntern(std::vector<trajectory_follower::SubTrajectory> const& value)
{
    p->data = value;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SubTrajectoryVisualization)

