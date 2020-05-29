#include "SubTrajectory.hpp"
#include <base/Float.hpp>
#include <base/Angle.hpp>
#include <boost/bind.hpp>

using namespace trajectory_follower;

std::pair<bool, double> oriFinder(double start_t, double end_t, double searchedValue, const base::geometry::Spline<1> &spline);

double SubTrajectory::angleLimit(double angle)
{
    if(angle > M_PI)
        return angle - 2*M_PI;
    else if (angle < -M_PI)
        return angle + 2*M_PI;
    else
        return angle;
}

SubTrajectory::SubTrajectory()
{
    driveMode = ModeAckermann;
    speed = 0.;
}

base::Trajectory SubTrajectory::toBaseTrajectory() const
{
    base::Trajectory tr;
    tr.spline = posSpline;
    tr.speed = this->speed;
    return tr;
}

void SubTrajectory::interpolate(const std::vector< base::Pose2D >& poses, const std::vector< double >& orientationDiff)
{
    if(poses.size() < 2)
        throw std::runtime_error("SubTrajectory::interpolate() given vector contains less than 2 points, interpolation is not possible");

    if(orientationDiff.size() != poses.size())
        throw std::runtime_error("SubTrajectory::interpolate(): Error size mismatch of poses and orientationDiff vector");

    startPose = poses.front();
    goalPose = poses.back();

    double diffSum = 0;
    int cnt = 0;
    std::vector<base::Vector3d> posV;
    std::vector<double> orientationLinearized;

    for(size_t i = 0; i < poses.size(); i++)
    {
        const base::Pose2D &pose(poses[i]);
        double diff = orientationDiff[i];

        diffSum += diff;

        if(diffSum > 2*M_PI)
        {
            int cntDiff = diffSum / (2*M_PI);
            cnt += cntDiff;
            diffSum -= cntDiff * 2*M_PI;
        }

        if(diffSum < 2*M_PI)
        {
            int cntDiff = diffSum / (2*M_PI);
            cnt -= cntDiff;
            diffSum += cntDiff * 2*M_PI;
        }

        double curOrientation = pose.orientation + cnt * 2*M_PI;
        posV.push_back(Eigen::Vector3d(pose.position.x(), pose.position.y(), 0.));
        orientationLinearized.push_back(curOrientation);
    }

    std::vector<double> params;
    posSpline.interpolate(posV, params);

    std::vector<double> params2;
    orientationSpline.interpolate(orientationLinearized, params2, params);
}

void SubTrajectory::interpolate(const std::vector< base::Pose2D >& poses)
{
    if(poses.size() < 2)
        throw std::runtime_error("SubTrajectory::interpolate() given vector contains less than 2 points, interpolation is not possible");

    startPose = poses.front();
    goalPose = poses.back();

    std::vector<base::Vector3d> posV;
    std::vector<double> orientationLinearized;

    double lastOrientation = base::NaN<double>();
    int curOffset = 0;

    for(const base::Pose2D pose: poses)
    {
        double curOrientation = pose.orientation + curOffset * 2*M_PI;

        //linearize orientation
        if(!base::isNaN<double>(lastOrientation))
        {
            double diff1;
            double secondOri = pose.orientation;
            int secOffset = curOffset;

            //always take smaller diff
            if(curOrientation > lastOrientation)
            {
                diff1 = curOrientation - lastOrientation;
            }
            else
            {
                diff1 = lastOrientation - curOrientation;
            }

            if(diff1 < 0)
            {
                secOffset = curOffset - 1;
            }
            else
            {
                secOffset = curOffset + 1;
            }

            secondOri = pose.orientation + (secOffset) * 2*M_PI;

            double diff2 = fabs(secondOri - lastOrientation);
            if(diff2 < diff1)
            {
                curOrientation = secondOri;
                curOffset = secOffset;
            }
            else
            {
                //nothing to do, offset does not change
            }
        }

        posV.push_back(Eigen::Vector3d(pose.position.x(), pose.position.y(), 0.));
        orientationLinearized.push_back(curOrientation);
        lastOrientation = curOrientation;
    }

    std::vector<double> params;
    posSpline.interpolate(posV, params, std::vector<double>());

    std::vector<double> params2;
    orientationSpline.interpolate(orientationLinearized, params2, params);
}

void SubTrajectory::interpolate(base::Pose2D start, const std::vector< base::Angle >& angles)
{
    startPose = start;
    goalPose = start;
    goalPose.orientation = angles.back().getRad();

    posSpline.setSingleton(Eigen::Vector3d(start.position.x(), start.position.y(), 0.));
    std::vector<double> anglesd;
    for(const base::Angle &a: angles)
    {
        anglesd.push_back(a.getRad());
    }

    orientationSpline.interpolate(anglesd);
}

SubTrajectory::SubTrajectory(const base::Trajectory& trajectory)
{
    posSpline = trajectory.spline;
    speed = trajectory.speed;
    startPose = getIntermediatePoint(posSpline.getStartParam());
    goalPose = getIntermediatePoint(posSpline.getEndParam());
    driveMode = ModeAckermann;
}

double SubTrajectory::advance(double curveParam, double length)
{
    if ((length < 0 && getDist(getStartParam(), curveParam) > std::fabs(length)) || (length > 0 && getDist(curveParam, getEndParam()) > length)) {
        return posSpline.advance(curveParam, length, posSpline.getGeometricResolution()).first;
    } else if (length == 0) {
        return curveParam;
    } else if (length < 0) {
        return getStartParam();
    } else {
        return getEndParam();
    }
}

std::pair< double, double > SubTrajectory::error(const Eigen::Vector2d& pos, double currentHeading, double curveParam, double forwardDist)
{
    double targetCurveParam = curveParam;
    Eigen::Vector2d targetPose = pos;
    double heading = currentHeading;
    if(!driveForward())
        heading = angleLimit(currentHeading + M_PI);
    double forwardLen = std::max(std::abs(forwardDist), 0.01);
    forwardLen = copysign(forwardLen, forwardDist);
    targetCurveParam = advance(targetCurveParam, std::abs(forwardLen));
    targetPose += Eigen::Rotation2Dd(currentHeading) * Eigen::Vector2d(forwardLen, 0.);
    targetCurveParam = getClosestPoint(base::Pose2D(targetPose, currentHeading), targetCurveParam, advance(curveParam, -std::abs(forwardLen)), advance(targetCurveParam, std::abs(forwardLen)));
    double distanceError = posSpline.distanceError(Eigen::Vector3d(pos.x(), pos.y(), 0.), curveParam);
    double headingError = posSpline.headingError(heading, curveParam);
        
    return std::make_pair(distanceError, headingError);
}

double SubTrajectory::getClosestPoint(const base::Pose2D& pose) const
{
    return getClosestPoint(pose, getStartParam());
}

double SubTrajectory::getClosestPoint(const base::Pose2D& pose, double guess) const
{
    if(!orientationSpline.isEmpty() && posSpline.isSingleton())
    {
        std::pair<bool, double> ret = oriFinder(getStartParam(), getEndParam(), pose.orientation, orientationSpline);
        if(!ret.first)
        {
            const base::geometry::Spline<1>::vector_t start = orientationSpline.getPoint(getStartParam());
            const base::geometry::Spline<1>::vector_t end = orientationSpline.getPoint(getEndParam());

            base::Angle searched = base::Angle::fromRad(pose.orientation);
            base::Angle startA = base::Angle::fromRad(start.x());
            base::Angle endA = base::Angle::fromRad(end.x());

            double diffToEnd = fabs((endA-searched).getRad());
            double diffToStart = fabs((startA-searched).getRad());

            if(diffToStart < diffToEnd)
                return getStartParam();
            else
                return getEndParam();
        }

        if(guess - 0.1 > getStartParam())
            guess -= 0.1;

        return orientationSpline.dichotomic_search(guess, getEndParam(), boost::bind(oriFinder, _1, _2, pose.orientation, _3), 0.0001, 0.0001).second;
    }

    return posSpline.findOneClosestPoint(Eigen::Vector3d(pose.position.x(), pose.position.y(), 0.), guess, getGeometricResolution());
}

double SubTrajectory::getClosestPoint(const base::Pose2D& pose, double guess, double start, double end)
{
    return posSpline.localClosestPointSearch(Eigen::Vector3d(pose.position.x(), pose.position.y(), 0.), guess, start, end);
}

double SubTrajectory::getCurvature(double param)
{
    return posSpline.getCurvature(param);
}

double SubTrajectory::getCurvatureMax()
{
    return posSpline.getCurvatureMax();
}

double SubTrajectory::getDist(double startParam, double endParam) const
{
    return posSpline.getCurveLength(startParam, endParam, posSpline.getGeometricResolution());
}

double SubTrajectory::getDistToGoal(double startParam) const
{
    if(!orientationSpline.isEmpty() && posSpline.isSingleton())
    {
        const base::geometry::Spline<1>::vector_t start = orientationSpline.getPoint(startParam);
        const base::geometry::Spline<1>::vector_t end = orientationSpline.getPoint(getEndParam());

        return fabs(start.x() - end.x());

    }

    return posSpline.getCurveLength(startParam, getEndParam(), posSpline.getGeometricResolution());
}

double SubTrajectory::getEndParam() const
{
    return posSpline.getEndParam();
}

const base::Pose2D& SubTrajectory::getGoalPose() const
{
    return goalPose;
}

base::Pose2D SubTrajectory::getIntermediatePoint(double d)
{
    base::Vector2d point;
    if(posSpline.isSingleton())
    {
        point = startPose.position;
    }
    else
    {
        auto p = posSpline.getPoint(d);
        point = Eigen::Vector2d(p.x(), p.y());
    }

    double orientation = 0.;
    if (!orientationSpline.isEmpty())
    {
        orientation = orientationSpline.getPoint(d).x();
    }
    else
    {
        orientation = posSpline.getHeading(d);
    }
    return base::Pose2D(point, orientation);
}

base::Pose2D SubTrajectory::getIntermediatePointNormalized(double d)
{
    base::Pose2D ret = getIntermediatePoint(d);
    ret.orientation = base::Angle::fromRad(ret.orientation).rad;
    return ret;
}

const double& SubTrajectory::getSpeed() const
{
    return speed;
}

double SubTrajectory::getStartParam() const
{
    return posSpline.getStartParam();
}

const base::Pose2D& SubTrajectory::getStartPose() const
{
    return startPose;
}

double SubTrajectory::getVariationOfCurvature(double param)
{
    return posSpline.getVariationOfCurvature(param);
}

void SubTrajectory::setGeometricResolution(double geometricResolution)
{
    this->posSpline.setGeometricResolution(geometricResolution);
    this->orientationSpline.setGeometricResolution(geometricResolution);
}

double SubTrajectory::getGeometricResolution() const {
    return posSpline.getGeometricResolution();
}

bool SubTrajectory::driveForward() const {
    return speed >= 0;
}

void SubTrajectory::setSpeed(double speed) {
    this->speed = speed;
}

double SubTrajectory::splineHeading(double param) {
    return posSpline.getHeading(param);
}

std::pair<bool, double> oriFinder(double start_t, double end_t, double searchedValue, const base::geometry::Spline<1> &spline)
{
    std::pair<bool, double> ret;

    const base::geometry::Spline<1>::vector_t start = spline.getPoint(start_t);
    const base::geometry::Spline<1>::vector_t end = spline.getPoint(end_t);

    double resolution = fabs((base::Angle::fromRad(start.x()) - base::Angle::fromRad(end.x())).getRad());
    base::AngleSegment segment;
    if(start.x() < end.x())
    {
        segment = base::AngleSegment(base::Angle::fromRad(start.x()), end.x() - start.x());
    }
    else
    {
        segment = base::AngleSegment(base::Angle::fromRad(end.x()), start.x() - end.x());
    }
    if(segment.isInside(base::Angle::fromRad(searchedValue)))
    {
        return std::make_pair(true, resolution);
    }

    return std::make_pair(false, resolution);
}

Lateral::Lateral()
    : SubTrajectory()
{
}

Lateral::Lateral(const base::Pose2D& currentPose, double angle, double length, double speed)
    : SubTrajectory()
{
    std::vector< base::Pose2D > poses;
    base::Pose2D endPose;
    endPose.orientation = currentPose.orientation;
    endPose.position = Eigen::Rotation2Dd(angle) * Eigen::Vector2d(length, 0.);
    poses.push_back(currentPose);
    poses.push_back(endPose);
    interpolate(poses);
    driveMode = ModeDiagonal;
    this->speed = speed;
}

Lateral::Lateral(const base::Pose2D& currentPose, const base::Position2D& end, double speed)
    : SubTrajectory()
{
    std::vector< base::Pose2D > poses;
    base::Pose2D endPose;
    endPose.orientation = currentPose.orientation;
    endPose.position = end;
    poses.push_back(currentPose);
    poses.push_back(endPose);
    interpolate(poses);
    driveMode = ModeDiagonal;
    this->speed = speed;
}