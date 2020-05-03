/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file    Line3.h
 * @brief   4 dimensional manifold of 3D lines
 * @author  Akshay Krishnan
 * @author  Frank Dellaert
 */
// \callgraph

#pragma once

#include <gtsam/base/concepts.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/slam/expressions.h>

#include <boost/random.hpp>
#include <cstdlib>
#include <ctime>

namespace gtsam{

/**
 * A 3D line (R,a,b) : (Rot3,Scalar,Scalar)
 * @addtogroup geometry
 * \nosubgrouping
 */
class Line3 {
private:
    Rot3 R_;    // Rotation of line about x and y in world frame
    double a_, b_;  // Intersection of line with x-y world plane

public:
    enum {dimension = 4};

    /** Default constructor is the Z axis **/
    Line3() :
            a_(0), b_(0)
    {}

    /** Constructor:
     * Parallel to z axis, intersecting x-y plane at (a,b) **/
    Line3(const double a, const double b):
            a_(a), b_(b)
    {}

    /** Constructor for general line from (R, a, b) **/
    Line3(const Rot3 &R, const double a, const double b) :
            R_(R), a_(a), b_(b)
    {}

    /** Sample two points lying on the line
     * @return Vector6 where first 3 elements are one point, and
     * next 3 elements are another point **/
    Vector6 points();

    /**
     * The retract method maps from the tangent space back to the manifold.
     * The tangent space for the rotation of a line is only two dimensional -
     * rotation about x and y
     * @param v: increment in tangent space
     * @param H: Jacobian of retraction with respect to the increment
     * @return: resulting line after adding the increment and mapping to the manifold
     */
    Line3 retract(const Vector4 &v, OptionalJacobian<4,4> H = boost::none) const;

    /**
     * The localCoordinates method is the inverse of retract and finds the difference
     * between two lines in the tangent space.
     * @param q Line3 on manifold
     * @param H OptionalJacobian of localCoordinates with respect to line
     * @return Vector4 difference in the tangent space
     */
    Vector4 localCoordinates(const Line3 &q, OptionalJacobian<4,4> H = boost::none) const;

    /**
     * Rotation of line accessor
     * @return Rot3
     */
    Rot3 R() const
    {
        return R_;
    }

    /**
     * Accessor for a, b
     * @return Vector2(a, b)
     */
    Vector2 V() const
    {
        return Vector2(a_, b_);
    }

    /**
     * Print R, a, b
     * @param s: optional starting string
     */
    void print(const std::string &s = "") const;

    /**
     * Check if two lines are equal
     * @param l2 - line to be compared
     * @param tol : optional tolerance
     * @return boolean - true if lines are equal
     */
    bool equals(const Line3 &l2, double tol = 10e-9) const;

    /**
     * Projecting a line to an image plane
     * @param Dline: OptionalJacobian of projected line with respect to this line
     * @return Point3 - projected line in image plane
     */
    Point3 project(OptionalJacobian<3, 4> Dline = boost::none) const;
};

template <>
struct traits<Line3> : public internal::Manifold<Line3> {};

template <>
struct traits<const Line3> : public internal::Manifold<Line3> {};

// For expression factor graphs
typedef Expression<Line3> Line3_;

/**
 * Function projects input line to image plane
 * @param L input line
 * @param Dline OptionalJacbian of projected line with respect to input line
 * @return Point3 - projected line
 */
Point3 projectLine(const Line3 &L,
        OptionalJacobian<3, 4> Dline = boost::none);

/**
 * Transform a line from world to camera frame
 * @param p - Pose3 of camera in world frame
 * @param l - Line3 in world frame
 * @param Dpose - OptionalJacobian of transformed line with respect to p
 * @param Dline -  OptionalJacobian of transformed line with respect to l
 * @return Transformed line in camera frame
 */
Line3 transformTo(const Pose3 &p, const Line3 &l,
                    OptionalJacobian<4, 6> Dpose = boost::none,
                    OptionalJacobian<4, 4> Dline = boost::none);

/**
 * Expression method for projection
 * @param line Line3_ expression
 * @return Point3_ projected line
 */
inline Point3_ projectLineExpression(const Line3_& line)
{
    Point3 (*f)(const Line3&, OptionalJacobian<3, 4>) = &projectLine;
    return Point3_(f, line);
}

/**
 * Expression method for transforming line
 * @param p - Pose3_ expression
 * @param line - Line3 expression
 * @return transformed Line3_ expression
 */
inline Line3_ transformToExpression(const Pose3_& p, const Line3_& line)
{
    Line3 (*f)(const Pose3&, const Line3&,
                OptionalJacobian<4,6>, OptionalJacobian<4,4>) = &transformTo;
    return Line3_(f, p, line);
}

}