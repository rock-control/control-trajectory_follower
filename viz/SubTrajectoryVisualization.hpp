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
    public:
        SubTrajectoryVisualization();
        ~SubTrajectoryVisualization();

    Q_INVOKABLE void updateData(std::vector<trajectory_follower::SubTrajectory> const &sample)
    {vizkit3d::Vizkit3DPlugin<std::vector<trajectory_follower::SubTrajectory>>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(std::vector<trajectory_follower::SubTrajectory> const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
