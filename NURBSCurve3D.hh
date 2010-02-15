/*
 * =====================================================================================
 *
 *       Filename:  NURBSCurve3D.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  11/27/09 14:50:04
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Ajish Babu (), ajish.babu@dfki.de
 *        Company:  DFKI
 *
 * =====================================================================================
 */

#ifndef  _CUBIC_CURVE3D_HPP_INC
#define  _CUBIC_CURVE3D_HPP_INC

#include <vector>
#include <eigen2/Eigen/Core>
#include "sisl.h"

namespace geometry
{
    /** Generic template implementation of a N-order NURBS, wrapping the sisl
     * library
     *
     * It offers an interface to interpolate around a certain number of points
     * using NURBS curves.
     */
    class NURBSCurve3D
    {
            static const int DIM = 3;
            NURBSCurve3D(SISLCurve* curve, double geometric_resolution, double order);

	public:
	    explicit NURBSCurve3D(double geometric_resolution = 0, double order = 3);
	    ~NURBSCurve3D();

            /** Changes the default geometric resolution */
            void setGeometricResolution(double _geores) { geometric_resolution = _geores; }

            /** Returns the number of points for this curve */
	    int    getPointCount() const { return points.size(); };
            /** Returns the length of the curve in geometric space */
	    double getCurveLength() const { return curve_length; }; 
	    double getStartParam() const { return start_param; };
	    double getEndParam()   const { return end_param; };

            /** Returns the length-to-parametric scale
             *
             * I.e. it returns the number of parametric units that lie in one
             * curve length unit
             */
	    double getUnitParameter() const { return (end_param - start_param) / curve_length;  };
   	  
            /** Returns the geometric point that lies on the curve at the given
             * parameter */
	    Eigen::Vector3d getPoint(double _param);

            /** Returns the curvature at the given position
             *
             * @throws out_of_range if _param is not in [start_param,
             * end_param] and runtime_error if SISL returns an error
             */
            double getCurvature(double _param);

            /** Returns the first order derivative of the curvature at the given
             * position
             *
             * @throws out_of_range if _param is not in [start_param,
             * end_param] and runtime_error if SISL returns an error
             */
	    double getVariationOfCurvature(double _param);  // Variation of Curvature
	 

            /** \overload
             */
            double findOneClosestPoint(Eigen::Vector3d const& _pt);

            /** Returns a single closest point to _pt
             *
             * This is a convenience method that calls findClosestPoints
             *
             * @return the parameter of the found closes point
             * @throw std::runtime_error if no points have been found (should not happen)
             * @see localClosestPointSearch findClosestPoints
             */
            double findOneClosestPoint(Eigen::Vector3d const& _pt, double _geometric_resolution);

            /** \overload
             */
            void findClosestPoints(Eigen::Vector3d const& _pt,
                    std::vector<double>& _points,
                    std::vector< std::pair<double, double> >& _curves);

            /**
             * Returns the single points and curve segments that are closest to
             * the given point.
             *
             */
            void findClosestPoints(Eigen::Vector3d const& _pt,
                    std::vector<double>& _points,
                    std::vector< std::pair<double, double> >& _curves,
                    double _geores);

            /** \overload
             */
            double localClosestPointSearch(Eigen::Vector3d const& _pt, double _guess, double _start, double _end);

            /** Performs a Newton search in the provided parametric interval, starting with the given guess.
             *
             * This method is subject to local minima problems
             */
            double localClosestPointSearch(Eigen::Vector3d const& _pt, double _guess, double _start, double _end, double _geores);

	    /** Computes the Frenet frame at a particular parameter */
	    Eigen::Matrix3d getFrenetFrame(double _param);

            /** Returns the curve heading at the given parametric position
             *
             * The heading is defined as the angle between the tangent projected
             * on the X-Y axis and the X axis itself.
             */
            double getHeading(double _param);

	    void printCurveProperties();

	    /** Calculates the pose error */
	    Eigen::Vector3d poseError(Eigen::Vector3d _pt, double _actZRot, double _start_param, double _length_tol);

	    /** Add pointst to generate the curve */
	    void addPoint(Eigen::Vector3d pt);

	    /** Generates the curve */
	    void update();

            /** Reinitializes the curve */
            void clear();

            std::vector<double> simplify();
            std::vector<double> simplify(double tolerance);

	private:
            //! the underlying SISL curve
	    SISLCurve *curve;
            //! the list of points from which we generated the curve
	    std::vector<Eigen::Vector3d> points;

            //! the geometric resolution
            double geometric_resolution;
            //! the order of the NURBS curve that approximates the points
            double curve_order;
            //! the start parameter (usually 0.0)
	    double start_param;
            //! the end parameter, as returned by SISL
	    double end_param;
            //! the length of the curve in geometric space
	    double curve_length; // Length of the curve
    };

}
#endif   /* ----- #ifndef _CUBIC_CURVE3D_HPP_INC  ----- */
