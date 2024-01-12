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

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

namespace gtsam {

class Line3;

/**
 * Transform a line from world to camera frame
 * @param wTc - Pose3 of camera in world frame
 * @param wL - Line3 in world frame
 * @param Dpose - OptionalJacobian of transformed line with respect to p
 * @param Dline -  OptionalJacobian of transformed line with respect to l
 * @return Transformed line in camera frame
 */
GTSAM_EXPORT Line3 transformTo(const Pose3 &wTc, const Line3 &wL,
                  OptionalJacobian<4, 6> Dpose = {},
                  OptionalJacobian<4, 4> Dline = {});


/**
 * A 3D line (R,a,b) : (Rot3,Scalar,Scalar)
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Line3 {
 private:
  Rot3 R_;    // Rotation of line about x and y in world frame
  double a_, b_;  // Intersection of line with the world x-y plane rotated by R_
  // Also the closest point on line to origin
 public:
  enum { dimension = 4 };

  /** Default constructor is the Z axis **/
  Line3() :
      a_(0), b_(0) {}

  /** Constructor for general line from (R, a, b) **/
  Line3(const Rot3 &R, const double a, const double b) :
      R_(R), a_(a), b_(b) {}

  /**
   * The retract method maps from the tangent space back to the manifold.
   * The method q = p + v, where p is this line, v is an increment along
   * the tangent space and q is the resulting line.
   * The tangent space for the rotation of a line is only two dimensional -
   * rotation about x and y
   * @param v: increment in tangent space
   * @param Dp: increment of retraction with respect to this line
   * @param Dv: Jacobian of retraction with respect to the increment
   * @return q: resulting line after adding the increment and mapping to the manifold
   */
  Line3 retract(const Vector4 &v,
                OptionalJacobian<4, 4> Dp = {},
                OptionalJacobian<4, 4> Dv = {}) const;

  /**
   * The localCoordinates method is the inverse of retract and finds the difference
   * between two lines in the tangent space. It computes v = q - p where q is an input
   * line, p is this line and v is their difference in the tangent space.
   * @param q: Line3 on manifold
   * @param Dp: OptionalJacobian of localCoordinates with respect to this line
   * @param Dq: OptionalJacobian of localCoordinates with respect to this line
   * @return v: difference in the tangent space
   */
  Vector4 localCoordinates(const Line3 &q,
                           OptionalJacobian<4, 4> Dp = {},
                           OptionalJacobian<4, 4> Dq = {}) const;

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
   * Projecting a line to the image plane. Assumes this line is in camera frame.
   * @param Dline: OptionalJacobian of projected line with respect to this line
   * @return Unit3 - projected line in image plane, in homogenous coordinates.
   * We use Unit3 since it is a manifold with the right dimension.
   */
  Unit3 project(OptionalJacobian<2, 4> Dline = {}) const;

  /**
   * Returns point on the line that is at a certain distance starting from the
   * point where the rotated XY axis intersects the line.
   * @param distance (can be positive or negative) - positive is the positive
   * direction of the rotated z axis that forms the line. The default value of zero
   * returns the point where the rotated XY axis intersects the line.
   * @return Point on the line
   */
  Point3 point(double distance = 0) const;

  /**
   * Return the rotation of the line.
   */
  inline Rot3 R() const {
    return R_;
  }

  /**
   * Return the x-coordinate of the intersection of the line with the xy plane.
   */
  inline double a() const {
    return a_;
  }

  /**
   * Return the y-coordinate of the intersection of the line with the xy plane.
   */
  inline double b() const {
    return b_;
  }

  /**
   * Transform a line from world to camera frame
   * @param wTc - Pose3 of camera in world frame
   * @param wL - Line3 in world frame
   * @param Dpose - OptionalJacobian of transformed line with respect to p
   * @param Dline -  OptionalJacobian of transformed line with respect to l
   * @return Transformed line in camera frame
   */
  GTSAM_EXPORT friend Line3 transformTo(const Pose3 &wTc, const Line3 &wL,
                           OptionalJacobian<4, 6> Dpose,
                           OptionalJacobian<4, 4> Dline);
};

template<>
struct traits<Line3> : public internal::Manifold<Line3> {};

template<>
struct traits<const Line3> : public internal::Manifold<Line3> {};
}