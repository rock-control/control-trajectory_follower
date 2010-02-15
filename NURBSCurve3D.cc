/*
 * =====================================================================================
 *
 *       Filename:  NURBNURBSSCurve3D.cpp
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  11/27/09 14:49:44
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu (), ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#include "NURBSCurve3D.hh"

#include <stdexcept>
#include <vector>

using namespace std;
using namespace geometry;
using namespace Eigen;

NURBSCurve3D::NURBSCurve3D(SISLCurve* curve, double geometric_resolution, double order)
    : curve(curve)
    , geometric_resolution(geometric_resolution)
    , curve_order(order) {}
NURBSCurve3D::NURBSCurve3D ( double _geometric_resolution, double _curve_order)
    : curve(0)
    , geometric_resolution(_geometric_resolution)
    , curve_order(_curve_order)
{
}

NURBSCurve3D::~NURBSCurve3D ()
{
    // Frees the memory of the curve 
    //if (curve)
    //    freeCurve(curve);
}

Vector3d NURBSCurve3D::getPoint(double _param)
{
    if (_param < start_param || _param > end_param) 
        throw std::out_of_range("_param is not in the [start_param, end_param] range");

    int leftknot; // Not needed
    double pos[DIM]; // Array to store the position
    int status;
    s1227(curve, 0, _param, &leftknot, pos, &status); // Gets the point
    if (status != 0)
        throw std::runtime_error("SISL error while computing a curve point");

    return Vector3d(pos[0], pos[1], pos[2]);
}

double NURBSCurve3D::getCurvature(double _param)
{
    // Limits the input paramter to the curve limit
    if (_param < start_param || _param > end_param) 
        throw std::out_of_range("_param is not in the [start_param, end_param] range");

    double curvature; 
    int status;
    s2550(curve, &_param, 1, &curvature, &status); // Gets the point
    if (status != 0)
        throw std::runtime_error("SISL error while computing a curvature");

    return curvature;
}

double NURBSCurve3D::getVariationOfCurvature(double _param)  // Variation of Curvature
{
    if (_param < start_param || _param > end_param) 
        throw std::out_of_range("_param is not in the [start_param, end_param] range");

    double VoC; 
    int status;
    s2556(curve, &_param, 1, &VoC, &status); // Gets the point
    if (status != 0)
        throw std::runtime_error("SISL error while computing a variation of curvature");

    return VoC;
}


void NURBSCurve3D::addPoint(Vector3d pt)
{
    points.push_back(pt);
}

void NURBSCurve3D::update()
{
    vector<double> points;
    points.reserve(getPointCount() * 3);
    vector<int> point_types;
    point_types.reserve(getPointCount());

    // Puts all the points into an array
    for (vector<Vector3d>::iterator it = this->points.begin(); it != this->points.end(); ++it) 
    {
	points.push_back(it->x());
	points.push_back(it->y());
	points.push_back(it->z());

        point_types.push_back(1); // ordinary point
    }

    /* 
       Input Arguments:
       points - Array (of length DIM inbpnt) containing the points/derivatives
       to be interpolated.
       nb_point - No. of points/derivatives in the epoint array.
       idim - The dimension of the space in which the points lie.
       nptyp - Array (length inbpnt) containing type indicator for 
       points/derivatives/second-derivatives:
       = 1 : Ordinary point.
       = 2 : Knuckle point. (Is treated as an ordinary
       point.)
       = 3 : Derivative to next point.
       = 4 : Derivative to prior point.
       = 5 : Second-derivative to next point.
       = 6 : Second derivative to prior point.
       = 13 : Point of tangent to next point.
       = 14 : Point of tangent to prior point.
       icnsta - Additional condition at the start of the curve:
       = 0 : No additional condition.
       = 1 : Zero curvature at start.
       icnend - Additional condition at the end of the curve:
       = 0 : No additional condition.
       = 1 : Zero curvature at end.
       iopen - Flag telling if the curve should be open or closed:
       = 1 : Open curve.
       = 0 : Closed, non-periodic curve.
       = -1 : Periodic (and closed) curve.
       ORDER - The order of the spline curve to be produced.
       start_param - Parameter value to be used at the start of the curve.

       Output Arguments:
       end_param - Parameter value used at the end of the curve.
       curve - Pointer to output B-spline curve.
       point_param - Pointer to the parameter values of the points in the 
       curve. Represented only once, although derivatives
       and second-derivatives will have the same parameter
       value as the points.
       nb_unique_param - No. of unique parameter values.
       status - Status message
       < 0 : Error.
       = 0 : Ok.
       > 0 : Warning.
       */

    // Generates curve
    double* point_param;  
    int nb_unique_param;
    start_param = 0.0;

    if (curve)
    {
        freeCurve(curve);
        curve = 0;
    }

    int status;
    s1356(&points[0], getPointCount(), DIM, &point_types[0], 0, 0, 1, curve_order, start_param, &end_param, &curve, 
	    &point_param, &nb_unique_param, &status);
    if (status != 0)
        throw std::runtime_error("cannot generate the curve");

    // Get the curve length
    s1240(curve, geometric_resolution, &curve_length, &status);
    if (status != 0)
        throw std::runtime_error("cannot get the curve length");
}

void NURBSCurve3D::printCurveProperties()
{
    std::cout << "CURVE PROPERTIES " << std::endl
	<< "  Order        : " << curve->ik    << std::endl
	<< "  Dimension    : " << curve->idim  << std::endl
	<< "  Kind         : " << curve->ikind << std::endl
	<< "  Parameters   : " << start_param  << "->" << end_param << std::endl
	<< "  Length       : " << curve_length << std::endl;
}

double NURBSCurve3D::findOneClosestPoint(Vector3d const& _pt)
{ return findOneClosestPoint(_pt, geometric_resolution); }

double NURBSCurve3D::findOneClosestPoint(Vector3d const& _pt, double _geores)
{
    vector<double> points;
    vector< pair<double, double> > curves;
    findClosestPoints(_pt, points, curves, _geores);
    if (points.empty())
    {
        if (curves.empty())
            throw std::logic_error("no closes point returned by findClosestPoints");
        return curves.front().first;
    }
    else
        return points.front();
}

void NURBSCurve3D::findClosestPoints(Vector3d const& _pt, vector<double>& _points, vector< pair<double, double> >& _curves)
{
    return findClosestPoints(_pt, _points, _curves, geometric_resolution);
}

void NURBSCurve3D::findClosestPoints(Vector3d const& _pt, vector<double>& _result_points, vector< pair<double, double> >& _result_curves, double _geores)
{
    double point[DIM] = {_pt.x(),_pt.y(),_pt.z() }; // Array of the point

    int points_count;
    double* points;
    int curves_count;
    SISLIntcurve** curves;

    // Finds the closest point on the curve
    int status;
    s1953(curve, point, DIM, _geores, _geores, &points_count, &points, &curves_count, &curves, &status);
    if (status != 0)
        throw std::runtime_error("failed to find the closest points");

    for (int i = 0; i < curves_count; ++i)
        _result_curves.push_back(make_pair(curves[i]->epar1[0], curves[i]->epar1[1]));
    for (int i = 0; i < points_count; ++i)
        _result_points.push_back(points[i]);

    free(curves);
    free(points);
}

double NURBSCurve3D::localClosestPointSearch(Vector3d const& _pt, double _guess, double _start, double _end)
{ return localClosestPointSearch(_pt, _guess, _start, _end, geometric_resolution); }

double NURBSCurve3D::localClosestPointSearch(Vector3d const& _pt, double _guess, double _start, double _end, double  _geores)
{
    double param;
    double point[DIM] = { _pt.x(),_pt.y(),_pt.z() }; // Array of the point

    // Finds the closest point on the curve
    int status;
    s1774(curve, point, DIM, _geores, _start, _end, _guess, &param, &status);
    if (status != 0)
        throw std::runtime_error("failed to find the closest points");

    // Returns the parameter of the point
    return param;
}

Matrix3d NURBSCurve3D::getFrenetFrame(double _param)
{
    double p;    // does nothing
    double t[3], n[3], b[3]; // Frame axis

    // Finds the frenet frame
    int status;
    s2559(curve, &_param, 1, &p, t, n, b, &status);

    // Writes the frame to a matrix
    Matrix3d frame;
    frame << t[0], t[1], t[2], n[0], n[1], n[2], b[0], b[1], b[2];

    return frame;
}

double NURBSCurve3D::getHeading(double _param)
{    
    Matrix3d frame = getFrenetFrame(_param);

    // Vector if the X axis of the frame
    Vector2d Xaxis(frame(0,0),frame(0,1));
    Xaxis.normalize(); 

    // Returns the angle of Frenet X axis in Inertial frame
    return atan2(Xaxis.y(),Xaxis.x());
}

void NURBSCurve3D::clear()
{
    if (curve)
    {
        freeCurve(curve);
        curve = 0;
    }
    points.clear();
}

vector<double> NURBSCurve3D::simplify()
{
    return simplify(geometric_resolution);
}

vector<double> NURBSCurve3D::simplify(double tolerance)
{
    if (!curve)
        throw std::runtime_error("the curve is not initialized");

    SISLCurve* result = NULL;
    double epsilon[3] = { tolerance, tolerance, tolerance };

    double maxerr[3];
    int status;
    s1940(curve, epsilon,
            curve_order, // derivatives
            curve_order, // derivatives
            1, // request closed curve
            10, // number of iterations
            &result, maxerr, &status);
    if (status != 0)
        throw std::runtime_error("SISL error while simplifying a curve");

    freeCurve(curve);
    curve = result;
    return vector<double>(maxerr, maxerr + 3);
}

