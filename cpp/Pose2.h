/**
 * @file  Pose2.h
 * @brief 2D Pose
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

// \callgraph

#pragma once

#include "Matrix.h"
#include "Testable.h"
#include "Lie.h"
#include "Point2.h"
#include "Rot2.h"

namespace gtsam {

  /**
   * A 2D pose (Point2,Rot2)
   */
  class Pose2: Testable<Pose2>, public Lie<Pose2>  {
  private:
    Point2 t_;
    Rot2 r_;

  public:

    /** default constructor = origin */
    Pose2() : t_(0.0, 0.0), r_(0) {} // default is origin

    /** copy constructor */
    Pose2(const Pose2& pose) : t_(pose.t_), r_(pose.r_) {}

    /**
     * construct from (x,y,theta)
     * @param x x coordinate
     * @param y y coordinate
     * @param theta angle with positive X-axis
     */
    Pose2(double x, double y, double theta) : t_(x,y), r_(theta) {}

    /** construct from rotation and translation */
    Pose2(double theta, const Point2& t) : t_(t), r_(theta) {}
    Pose2(const Rot2& r, const Point2& t) : t_(t), r_(r) {}

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** assert equality up to a tolerance */
    bool equals(const Pose2& pose, double tol = 1e-9) const;

    /** get functions for x, y, theta */
    double x()     const { return t_.x(); }
    double y()     const { return t_.y(); }
    double theta() const { return r_.theta(); }
    Point2 t()     const { return t_; }
    Rot2 r()       const { return r_; }

  }; // Pose2


  /** return DOF, dimensionality of tangent space = 3 */
  inline size_t dim(const Pose2&) { return 3; }

  /** inverse transformation */
  inline Pose2 inverse(const Pose2& pose) {
    return Pose2(inverse(pose.r()),
        pose.r().unrotate(Point2(-pose.t().x(), -pose.t().y()))); }

  /** compose this transformation onto another (pre-multiply this*p1) */
  inline Pose2 compose(const Pose2& p1, const Pose2& p0) {
    return Pose2(p0.r()*p1.r(), p0.t() + p0.r()*p1.t()); }

  /** exponential and log maps around identity */

  /** Create an incremental pose from x,y,theta */
  template<> inline Pose2 expmap(const Vector& v) { return Pose2(v[0], v[1], v[2]); }

  /** Return the x,y,theta of this pose */
  inline Vector logmap(const Pose2& p) { return Vector_(3, p.x(), p.y(), p.theta()); }

  /** print using member print function, currently used by LieConfig */
  inline void print(const Pose2& obj, const std::string& str = "") { obj.print(str); }

  /** Return point coordinates in pose coordinate frame */
  inline Point2 transform_to(const Pose2& pose, const Point2& point) {
    return unrotate(pose.r(), point-pose.t()); }
  Matrix Dtransform_to1(const Pose2& pose, const Point2& point);
  Matrix Dtransform_to2(const Pose2& pose, const Point2& point);

  /** Return point coordinates in global frame */
  inline Point2 transform_from(const Pose2& pose, const Point2& point) {
    return rotate(pose.r(), point)+pose.t(); }

  /** Return relative pose between p1 and p2, in p1 coordinate frame */
  /** todo: make sure compiler finds this version of between. */
  //inline Pose2 between(const Pose2& p0, const Pose2& p2) {
  //  return Pose2(p0.r().invcompose(p2.r()), p0.r().unrotate(p2.t()-p0.t())); }
  Matrix Dbetween1(const Pose2& p0, const Pose2& p2);
  Matrix Dbetween2(const Pose2& p0, const Pose2& p2);

  /** same as compose (pre-multiply this*p1) */
  inline Pose2 operator*(const Pose2& p1, const Pose2& p0) { return compose(p1, p0); }

  /** Transform a point in this coordinate frame to global coordinates,
   * same as transform_from */
  inline Point2 operator*(const Pose2& pose, const Point2& point) {
    return transform_from(pose, point); }



} // namespace gtsam

