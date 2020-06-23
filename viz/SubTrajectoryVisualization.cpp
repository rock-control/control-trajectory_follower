#include <iostream>
#include "SubTrajectoryVisualization.hpp"
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>
#include <osgViz/OsgViz.hpp>

#include <osg/LineWidth>

using namespace vizkit3d;
using trajectory_follower::DriveMode;

struct SubTrajectoryVisualization::Data {
    std::vector<trajectory_follower::SubTrajectory> data;
};


SubTrajectoryVisualization::SubTrajectoryVisualization()
    : p(new Data)
    , line_width( 4.0 )
    , rescueColor(1., 0., 0., 1.)
    , ackermannColor(1., 1., 0., 1.)
    , turnOnTheSpotColor(1., 0., 1., 1.)
    , sidewaysColor(0., 1., 0., 1.)
    , diagonalColor(0., 1., 1., 1.)
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
        
        osg::Vec4 currentColor;
        switch (traj.driveMode) {
            case DriveMode::ModeAckermann:
                currentColor = ackermannColor;
                break;
            case DriveMode::ModeTurnOnTheSpot:
                currentColor = turnOnTheSpotColor;
                break;
            case DriveMode::ModeSideways:
                currentColor = sidewaysColor;
                break;
            case DriveMode::ModeDiagonal:
                currentColor = diagonalColor;
                break;
        }
        if (traj.kind == trajectory_follower::TRAJECTORY_KIND_RESCUE)
            currentColor = rescueColor;
        
        auto prim = fac->createLinesNode(currentColor, osgPoints);
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
    // getColor and setColor currently have no real use as the trajectory is colored according
    // to its drive mode and not in a single predefined color.
    std::cerr << "Calling get/setColor() on SubTrajectoryVisualization is deprecated." << std::endl;
}

void SubTrajectoryVisualization::setRescueColor(QColor color)
{
    this->rescueColor = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("RescueColor");
    setDirty();
}

void SubTrajectoryVisualization::setAckermannColor(QColor color)
{
    this->ackermannColor = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("AckermannColor");
    setDirty();
}

void SubTrajectoryVisualization::setTurnOnTheSpotColor(QColor color)
{
    this->turnOnTheSpotColor = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("TurnOnTheSpotColor");
    setDirty();
}

void SubTrajectoryVisualization::setSidewaysColor(QColor color)
{
    this->sidewaysColor = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("SidewaysColor");
    setDirty();
}

void SubTrajectoryVisualization::setDiagonalColor(QColor color)
{
    this->diagonalColor = osg::Vec4(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    emit propertyChanged("DiagonalColor");
    setDirty();
}

QColor SubTrajectoryVisualization::getColor() const
{
    // getColor and setColor currently have no real use as the trajectory is colored according
    // to its drive mode and not in a single predefined color.
    std::cerr << "Calling get/setColor() on SubTrajectoryVisualization is deprecated." << std::endl;
    return QColor();
}

QColor SubTrajectoryVisualization::getRescueColor() const
{
    QColor c;
    c.setRgbF(rescueColor.x(), rescueColor.y(), rescueColor.z(), rescueColor.w());
    return c;
}

QColor SubTrajectoryVisualization::getAckermannColor() const
{
    QColor c;
    c.setRgbF(ackermannColor.x(), ackermannColor.y(), ackermannColor.z(), ackermannColor.w());
    return c;
}

QColor SubTrajectoryVisualization::getTurnOnTheSpotColor() const
{
    QColor c;
    c.setRgbF(turnOnTheSpotColor.x(), turnOnTheSpotColor.y(), turnOnTheSpotColor.z(), turnOnTheSpotColor.w());
    return c;
}

QColor SubTrajectoryVisualization::getSidewaysColor() const
{
    QColor c;
    c.setRgbF(sidewaysColor.x(), sidewaysColor.y(), sidewaysColor.z(), sidewaysColor.w());
    return c;
}

QColor SubTrajectoryVisualization::getDiagonalColor() const
{
    QColor c;
    c.setRgbF(diagonalColor.x(), diagonalColor.y(), diagonalColor.z(), diagonalColor.w());
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


//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SubTrajectoryVisualization)
