/**
 *@file  Pose3.h
 *@brief 3D Pose
 */

// \callgraph

#pragma once

#include <boost/numeric/ublas/vector_proxy.hpp>

#include "Point3.h"
#include "Rot3.h"
#include "Testable.h"
#include "Lie.h"

namespace gtsam {

  /** A 3D pose (R,t) : (Rot3,Point3) */
  class Pose3 : Testable<Pose3>, public Lie<Pose3> {
  private:
    Rot3 R_;
    Point3 t_;

  public:

    /** Default constructor is origin */
    Pose3() {}

    /** Copy constructor */
    Pose3(const Pose3& pose) : R_(pose.R_), t_(pose.t_) {}

    /** Construct from R,t */
    Pose3(const Rot3& R, const Point3& t) : R_(R), t_(t) {}

    /** Constructor from 4*4 matrix */
    Pose3(const Matrix &T) :
      R_(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0),
          T(2, 1), T(2, 2)), t_(T(0, 3), T(1, 3), T(2, 3)) {}

    /** Constructor from 12D vector */
    Pose3(const Vector &V) :
      R_(V(0), V(3), V(6), V(1), V(4), V(7), V(2), V(5), V(8)),
      t_(V(9), V(10),V(11)) {}

    const Rot3& rotation() const { return R_; }

    const Point3& translation() const { return t_; }

    /** convert to 4*4 matrix */
    Matrix matrix() const;

    /** print with optional string */
    void print(const std::string& s = "") const;

    /** assert equality up to a tolerance */
    bool equals(const Pose3& pose, double tol = 1e-9) const;

    Pose3 transform_to(const Pose3& pose) const;

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & BOOST_SERIALIZATION_NVP(R_);
      ar & BOOST_SERIALIZATION_NVP(t_);
    }
  }; // Pose3 class

  // global print
  inline void print(const Pose3& p, const std::string& s = "") { p.print(s);}

  // Dimensionality of the tangent space
  inline size_t dim(const Pose3&) { return 6; }

  // Compose two poses
  inline Pose3 compose(const Pose3& p0, const Pose3& p1) {
    return Pose3(p0.rotation()*p1.rotation(),
        p0.translation() + p0.rotation()*p1.translation());
  }

  // Find the inverse pose s.t. inverse(p)*p = Pose3()
  inline Pose3 inverse(const Pose3& p) {
    Rot3 Rt = inverse(p.rotation());
    return Pose3(Rt, Rt*(-p.translation()));
  }

  // Exponential map at identity - create a pose with a translation and
  // rotation (in canonical coordinates)
  template<> Pose3 expmap(const Vector& d);

  // Log map at identity - return the translation and canonical rotation
  // coordinates of a pose.
  Vector logmap(const Pose3& p);

  // todo: these are the "old-style" expmap and logmap about the specified
  // pose.
  // Increments the offset and rotation independently given a translation and
  // canonical rotation coordinates
  template<> inline Pose3 expmap<Pose3>(const Pose3& p0, const Vector& d) {
    return Pose3(expmap(p0.rotation(), sub(d, 0, 3)),
        expmap(p0.translation(), sub(d, 3, 6)));
  }

  // Independently computes the logmap of the translation and rotation.
  template<> inline Vector logmap<Pose3>(const Pose3& p0, const Pose3& pp) {
    const Vector r(logmap(p0.rotation(), pp.rotation())),
        t(logmap(p0.translation(), pp.translation()));
    return concatVectors(2, &r, &t);
  }


  /** receives the point in Pose coordinates and transforms it to world coordinates */
  Point3 transform_from(const Pose3& pose, const Point3& p);
  inline Point3 operator*(const Pose3& pose, const Point3& p) { return transform_from(pose, p); }
  Matrix Dtransform_from1(const Pose3& pose, const Point3& p);
  Matrix Dtransform_from2(const Pose3& pose); // does not depend on p !

  /** receives the point in world coordinates and transforms it to Pose coordinates */
  Point3 transform_to(const Pose3& pose, const Point3& p);
  Matrix Dtransform_to1(const Pose3& pose, const Point3& p);
  Matrix Dtransform_to2(const Pose3& pose, const Point3& p);

  /**
   * Derivatives of compose
   */
  Matrix Dcompose1(const Pose3& p1, const Pose3& p2);
  Matrix Dcompose2(const Pose3& p1, const Pose3& p2);

  /**
   * Derivative of inverse
   */
  Matrix Dinverse(const Pose3& p);

  /**
   * Return relative pose between p1 and p2, in p1 coordinate frame
   */
  Matrix Dbetween1(const Pose3& p1, const Pose3& p2);
  Matrix Dbetween2(const Pose3& p1, const Pose3& p2);

  /** direct measurement of a pose */
  Vector hPose(const Vector& x);

  /**
   * derivative of direct measurement
   * 12*6, entry i,j is how measurement error will change
   */
  Matrix DhPose(const Vector& x);

} // namespace gtsam
