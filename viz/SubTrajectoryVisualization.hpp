#pragma once

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <trajectory_follower/SubTrajectory.hpp>

namespace vizkit3d
{
    class SubTrajectoryVisualization
        : public vizkit3d::Vizkit3DPlugin<std::vector<trajectory_follower::SubTrajectory>>
        , boost::noncopyable
    {
    Q_OBJECT

    Q_PROPERTY(double LineWidth READ getLineWidth WRITE setLineWidth)
    Q_PROPERTY(QColor RescueColor READ getRescueColor WRITE setRescueColor)
    Q_PROPERTY(QColor AckermannColor READ getAckermannColor WRITE setAckermannColor)
    Q_PROPERTY(QColor TurnOnTheSpotColor READ getTurnOnTheSpotColor WRITE setTurnOnTheSpotColor)
    Q_PROPERTY(QColor SidewaysColor READ getSidewaysColor WRITE setSidewaysColor)
    Q_PROPERTY(QColor DiagonalColor READ getDiagonalColor WRITE setDiagonalColor)

    public:
        SubTrajectoryVisualization();
        ~SubTrajectoryVisualization();

    Q_INVOKABLE void updateData(std::vector<trajectory_follower::SubTrajectory> const &sample)
    {vizkit3d::Vizkit3DPlugin<std::vector<trajectory_follower::SubTrajectory>>::updateData(sample);}

    public slots:
        double getLineWidth();
        void setLineWidth(double line_width);
        void setColor(QColor color);
        QColor getColor() const;
        void setRescueColor(QColor color);
        QColor getRescueColor() const;
        void setAckermannColor(QColor color);
        QColor getAckermannColor() const;
        void setTurnOnTheSpotColor(QColor color);
        QColor getTurnOnTheSpotColor() const;
        void setSidewaysColor(QColor color);
        QColor getSidewaysColor() const;
        void setDiagonalColor(QColor color);
        QColor getDiagonalColor() const;

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(std::vector<trajectory_follower::SubTrajectory> const& plan);
        
    private:
        struct Data;
        Data* p;

        double line_width;
        osg::Vec4 rescueColor,
            ackermannColor,
            turnOnTheSpotColor,
            sidewaysColor,
            diagonalColor;

        osg::ref_ptr<osg::PositionAttitudeTransform> geode; 
    };
}