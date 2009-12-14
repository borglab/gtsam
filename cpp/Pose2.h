/**
 * @file  Pose2.h
 * @brief 2D Pose
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

// \callgraph

#pragma once

#include "Point2.h"
#include "Rot2.h"
#include "Matrix.h"
#include "Testable.h"

namespace gtsam {

/**
 * A 2D pose (Point2,Rot2)
 */
class Pose2: Testable<Pose2>  {

private:
  Point2 t_;
  Rot2 r_;

public:

  /** default constructor = origin */
  Pose2() :
    t_(0.0, 0.0), r_(0) { } // default is origin

  /** copy constructor */
  Pose2(const Pose2& pose) :
    t_(pose.t_), r_(pose.r_) { }

  /**
   * construct from (x,y,theta)
   * @param x x coordinate
   * @param y y coordinate
   * @param theta angle with positive X-axis
   */
  Pose2(double x, double y, double theta) :
    t_(x,y), r_(theta) { }

  /** construct from rotation and translation */
  Pose2(const Point2& t, double theta) :
    t_(t), r_(theta) { }
  Pose2(double theta, const Point2& t) :
    t_(t), r_(theta) { }
  Pose2(const Point2& t, const Rot2& r) :
    t_(t), r_(r) { }
  Pose2(const Rot2& r, const Point2& t) :
    t_(t), r_(r) { }

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** assert equality up to a tolerance */
  bool equals(const Pose2& pose, double tol = 1e-9) const;

  /** get functions for x, y, theta */
  double x()     const { return t_.x();}
  double y()     const { return t_.y();}
  double theta() const { return r_.theta();}
  Point2 t()     const { return t_; }
  Rot2 r()       const { return r_; }

  /** return DOF, dimensionality of tangent space = 3 */
  size_t dim() const { return 3; }

  /* exponential map */
  Pose2 exmap(const Vector& v) const;

  /** return vectorized form (column-wise) */
  Vector vector() const;

  /** rotate pose by theta */
//  Pose2 rotate(double theta) const;

  /** inverse transformation */
  Pose2 inverse() const {
    return Pose2(r_, r_.unrotate(t_));
  }

  /** compose this transformation onto another (pre-multiply this*p1) */
  Pose2 compose(const Pose2& p1) const {
    return Pose2(p1.r_*r_, p1.r_*t_+p1.t_);
  }

  /** same as compose (pre-multiply this*p1) */
  Pose2 operator*(const Pose2& p1) const {
    return compose(p1);
  }

  /** Return point coordinates in pose coordinate frame, same as transform_to */
  Point2 operator*(const Point2& point) const {
    return r_.unrotate(point-t_);
  }

  // operators
  Pose2 operator+(const Pose2& p2) const {
    return Pose2(t_+p2.t_, r_*p2.r_);
  }

  Pose2 operator-(const Pose2& p2) const {
    return Pose2(t_-p2.t_, r_.invcompose(p2.r_));
  }
}; // Pose2

/**
 * Return point coordinates in pose coordinate frame
 */
Point2 transform_to(const Pose2& pose, const Point2& point);
Matrix Dtransform_to1(const Pose2& pose, const Point2& point);
Matrix Dtransform_to2(const Pose2& pose, const Point2& point);

/**
 * Return relative pose between p1 and p2, in p1 coordinate frame
 */
Pose2 between(const Pose2& p1, const Pose2& p2);
Matrix Dbetween1(const Pose2& p1, const Pose2& p2);
Matrix Dbetween2(const Pose2& p1, const Pose2& p2);

} // namespace gtsam
