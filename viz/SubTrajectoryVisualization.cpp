#include <iostream>
#include "SubTrajectoryVisualization.hpp"
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/OsgViz.hpp>

#include <osg/LineWidth>

using namespace vizkit3d;

struct SubTrajectoryVisualization::Data {
    std::vector<trajectory_follower::SubTrajectory> data;
};


SubTrajectoryVisualization::SubTrajectoryVisualization()
    : p(new Data), line_width( 4.0 ), color(1., 1., 0., 1.), rescueColor(1., 0., 0., 1.)
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
    geode = static_cast<osg::PositionAttitudeTransform*>(node);
    geode->removeChildren(0, geode->getNumChildren());

    std::shared_ptr<osgviz::PrimitivesFactory> fac = osgviz::OsgViz::getInstance()->getModuleInstance<osgviz::PrimitivesFactory>("PrimitivesFactory");
    
    for(const trajectory_follower::SubTrajectory& traj : p->data)
    {
        std::vector<osg::Vec3> osgPoints;
        const base::geometry::Spline<3>& spline = traj.posSpline;
        double stepSize = (spline.getEndParam() - spline.getStartParam()) / (spline.getCurveLength() / 0.05);
        for(double param = spline.getStartParam(); param <= spline.getEndParam(); param += stepSize )
        {
            const Eigen::Vector3d splinePoint = spline.getPoint(param);
            osgPoints.emplace_back(splinePoint.x(), splinePoint.y(), splinePoint.z());
        }
        
        osg::Vec4 currentColor = traj.kind == trajectory_follower::TRAJECTORY_KIND_RESCUE? rescueColor : color;
        
        auto prim = fac->createLinesNode(color, osgPoints);
        geode->addChild(prim);
        
    }
    
    osg::StateSet* stateset = geode->getOrCreateStateSet();
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(line_width);
    stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);    
}

void SubTrajectoryVisualization::updateDataIntern(std::vector<trajectory_follower::SubTrajectory> const& value)
{
    p->data = value;
}

void SubTrajectoryVisualization::setColor(QColor color)
{
    this->color = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("Color");
    setDirty();
}

void SubTrajectoryVisualization::setRescueColor(QColor color)
{
    this->rescueColor = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("RescueColor");
    setDirty();
}

QColor SubTrajectoryVisualization::getColor() const
{
    QColor c;
    c.setRgbF(color.x(), color.y(), color.z(), color.w());
    return c;
}

QColor SubTrajectoryVisualization::getRescueColor() const
{
    QColor c;
    c.setRgbF(rescueColor.x(), rescueColor.y(), rescueColor.z(), rescueColor.w());
    return c;
}

double SubTrajectoryVisualization::getLineWidth()
{
    return line_width;
}

void SubTrajectoryVisualization::setLineWidth(double line_width)
{
    this->line_width = line_width;
    if(geode)
    {
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        osg::LineWidth* linewidth = new osg::LineWidth();
        linewidth->setWidth(line_width);
        stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    }
    emit propertyChanged("LineWidth");
}


