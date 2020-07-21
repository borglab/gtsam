/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *@file  Pose3.h
 *@brief 3D Pose
 */

// \callgraph
#pragma once

#include <gtsam/config.h>

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Lie.h>

namespace gtsam {

class Pose2;
// forward declare

/**
 * A 3D pose (R,t) : (Rot3,Point3)
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Pose3: public LieGroup<Pose3, 6> {
public:

  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Translation;

private:

  Rot3 R_; ///< Rotation gRp, between global and pose frame
  Point3 t_; ///< Translation gPp, from global origin to pose frame origin

public:

  /// @name Standard Constructors
  /// @{

  /** Default constructor is origin */
 Pose3() : R_(traits<Rot3>::Identity()), t_(traits<Point3>::Identity()) {}

  /** Copy constructor */
  Pose3(const Pose3& pose) :
      R_(pose.R_), t_(pose.t_) {
  }

  /** Construct from R,t */
  Pose3(const Rot3& R, const Point3& t) :
      R_(R), t_(t) {
  }

  /** Construct from Pose2 */
  explicit Pose3(const Pose2& pose2);

  /** Constructor from 4*4 matrix */
  Pose3(const Matrix &T) :
      R_(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1),
          T(2, 2)), t_(T(0, 3), T(1, 3), T(2, 3)) {
  }

  /// Named constructor with derivatives
  static Pose3 Create(const Rot3& R, const Point3& t,
                      OptionalJacobian<6, 3> HR = boost::none,
                      OptionalJacobian<6, 3> Ht = boost::none);

  /**
   *  Create Pose3 by aligning two point pairs
   *  A pose aTb is estimated between pairs (a_point, b_point) such that a_point = aTb * b_point
   *  Note this allows for noise on the points but in that case the mapping will not be exact.
   */
  static boost::optional<Pose3> Align(const std::vector<Point3Pair>& abPointPairs);

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const Pose3& pose, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static Pose3 identity() {
    return Pose3();
  }

  /// inverse transformation with derivatives
  Pose3 inverse() const;

  /// compose syntactic sugar
  Pose3 operator*(const Pose3& T) const {
    return Pose3(R_ * T.R_, t_ + R_ * T.t_);
  }

  /// @}
  /// @name Lie Group
  /// @{

  /// Exponential map at identity - create a rotation from canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$
  static Pose3 Expmap(const Vector6& xi, OptionalJacobian<6, 6> Hxi = boost::none);

  /// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ of this rotation
  static Vector6 Logmap(const Pose3& pose, OptionalJacobian<6, 6> Hpose = boost::none);

  /**
   * Calculate Adjoint map, transforming a twist in the this pose's (i.e, body) frame to the world spatial frame
   * Ad_pose is 6*6 matrix that when applied to twist xi \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$, returns Ad_pose(xi)
   */
  Matrix6 AdjointMap() const; /// FIXME Not tested - marked as incorrect

  /**
   * Apply this pose's AdjointMap Ad_g to a twist \f$ \xi_b \f$, i.e. a body-fixed velocity, transforming it to the spatial frame
   * \f$ \xi^s = g*\xi^b*g^{-1} = Ad_g * \xi^b \f$
   */
  Vector6 Adjoint(const Vector6& xi_b) const {
    return AdjointMap() * xi_b;
  } /// FIXME Not tested - marked as incorrect

  /**
   * Compute the [ad(w,v)] operator as defined in [Kobilarov09siggraph], pg 11
   * [ad(w,v)] = [w^, zero3; v^, w^]
   * Note that this is the matrix representation of the adjoint operator for se3 Lie algebra,
   * aka the Lie bracket, and also the derivative of Adjoint map for the Lie group SE3.
   *
   * Let \f$ \hat{\xi}_i \f$ be the se3 Lie algebra, and \f$ \hat{\xi}_i^\vee = \xi_i = [\omega_i,v_i] \in \mathbb{R}^6\f$ be its
   * vector representation.
   * We have the following relationship:
   * \f$ [\hat{\xi}_1,\hat{\xi}_2]^\vee = ad_{\xi_1}(\xi_2) = [ad_{(\omega_1,v_1)}]*\xi_2 \f$
   *
   * We use this to compute the discrete version of the inverse right-trivialized tangent map,
   * and its inverse transpose in the discrete Euler Poincare' (DEP) operator.
   *
   */
  static Matrix6 adjointMap(const Vector6 &xi);

  /**
   * Action of the adjointMap on a Lie-algebra vector y, with optional derivatives
   */
  static Vector6 adjoint(const Vector6 &xi, const Vector6 &y,
      OptionalJacobian<6, 6> Hxi = boost::none);

  // temporary fix for wrappers until case issue is resolved
  static Matrix6 adjointMap_(const Vector6 &xi) { return adjointMap(xi);}
  static Vector6 adjoint_(const Vector6 &xi, const Vector6 &y) { return adjoint(xi, y);}

  /**
   * The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
   */
  static Vector6 adjointTranspose(const Vector6& xi, const Vector6& y,
      OptionalJacobian<6, 6> Hxi = boost::none);

  /// Derivative of Expmap
  static Matrix6 ExpmapDerivative(const Vector6& xi);

  /// Derivative of Logmap
  static Matrix6 LogmapDerivative(const Pose3& xi);

  // Chart at origin, depends on compile-time flag GTSAM_POSE3_EXPMAP
  struct ChartAtOrigin {
    static Pose3 Retract(const Vector6& xi, ChartJacobian Hxi = boost::none);
    static Vector6 Local(const Pose3& pose, ChartJacobian Hpose = boost::none);
  };

  using LieGroup<Pose3, 6>::inverse; // version with derivative

  /**
   * wedge for Pose3:
   * @param xi 6-dim twist (omega,v) where
   *  omega = (wx,wy,wz) 3D angular velocity
   *  v (vx,vy,vz) = 3D velocity
   * @return xihat, 4*4 element of Lie algebra that can be exponentiated
   */
  static Matrix wedge(double wx, double wy, double wz, double vx, double vy,
      double vz) {
    return (Matrix(4, 4) << 0., -wz, wy, vx, wz, 0., -wx, vy, -wy, wx, 0., vz, 0., 0., 0., 0.).finished();
  }

  /// @}
  /// @name Group Action on Point3
  /// @{

  /**
   * @brief takes point in Pose coordinates and transforms it to world coordinates
   * @param point point in Pose coordinates
   * @param Hself optional 3*6 Jacobian wrpt this pose
   * @param Hpoint optional 3*3 Jacobian wrpt point
   * @return point in world coordinates
   */
  Point3 transformFrom(const Point3& point, OptionalJacobian<3, 6> Hself =
      boost::none, OptionalJacobian<3, 3> Hpoint = boost::none) const;

  /** syntactic sugar for transformFrom */
  inline Point3 operator*(const Point3& point) const {
    return transformFrom(point);
  }

  /**
   * @brief takes point in world coordinates and transforms it to Pose coordinates
   * @param point point in world coordinates
   * @param Hself optional 3*6 Jacobian wrpt this pose
   * @param Hpoint optional 3*3 Jacobian wrpt point
   * @return point in Pose coordinates
   */
  Point3 transformTo(const Point3& point, OptionalJacobian<3, 6> Hself =
      boost::none, OptionalJacobian<3, 3> Hpoint = boost::none) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// get rotation
  const Rot3& rotation(OptionalJacobian<3, 6> Hself = boost::none) const;

  /// get translation
  const Point3& translation(OptionalJacobian<3, 6> Hself = boost::none) const;

  /// get x
  double x() const {
    return t_.x();
  }

  /// get y
  double y() const {
    return t_.y();
  }

  /// get z
  double z() const {
    return t_.z();
  }

  /** convert to 4*4 matrix */
  Matrix4 matrix() const;

  /** 
    * Assuming self == wTa, takes a pose aTb in local coordinates 
    * and transforms it to world coordinates wTb = wTa * aTb.
    * This is identical to compose.
    */
  Pose3 transformPoseFrom(const Pose3& aTb, OptionalJacobian<6, 6> Hself = boost::none,
                                            OptionalJacobian<6, 6> HaTb = boost::none) const;

  /** 
   *  Assuming self == wTa, takes a pose wTb in world coordinates 
   * and transforms it to local coordinates aTb = inv(wTa) * wTb 
   */
  Pose3 transformPoseTo(const Pose3& wTb, OptionalJacobian<6, 6> Hself = boost::none,
                                          OptionalJacobian<6, 6> HwTb = boost::none) const;

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @return range (double)
   */
  double range(const Point3& point, OptionalJacobian<1, 6> Hself = boost::none,
      OptionalJacobian<1, 3> Hpoint = boost::none) const;

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @return range (double)
   */
  double range(const Pose3& pose, OptionalJacobian<1, 6> Hself = boost::none,
      OptionalJacobian<1, 6> Hpose = boost::none) const;

  /**
   * Calculate bearing to a landmark
   * @param point 3D location of landmark
   * @return bearing (Unit3)
   */
  Unit3 bearing(const Point3& point, OptionalJacobian<2, 6> Hself = boost::none,
      OptionalJacobian<2, 3> Hpoint = boost::none) const;

  /**
   * Calculate bearing to another pose
   * @param other 3D location and orientation of other body. The orientation
   * information is ignored.
   * @return bearing (Unit3)
   */
  Unit3 bearing(const Pose3& pose, OptionalJacobian<2, 6> Hself = boost::none,
      OptionalJacobian<2, 6> Hpose = boost::none) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Return the start and end indices (inclusive) of the translation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  inline static std::pair<size_t, size_t> translationInterval() {
    return std::make_pair(3, 5);
  }

  /**
   * Return the start and end indices (inclusive) of the rotation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  static std::pair<size_t, size_t> rotationInterval() {
    return std::make_pair(0, 2);
  }

  /// Output stream operator
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const Pose3& p);

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
  /// @name Deprecated
  /// @{
  Point3 transform_from(const Point3& point,
                        OptionalJacobian<3, 6> Hself = boost::none,
                        OptionalJacobian<3, 3> Hpoint = boost::none) const {
    return transformFrom(point, Hself, Hpoint);
  }
  Point3 transform_to(const Point3& point,
                      OptionalJacobian<3, 6> Hself = boost::none,
                      OptionalJacobian<3, 3> Hpoint = boost::none) const {
    return transformTo(point, Hself, Hpoint);
  }
  Pose3 transform_pose_to(const Pose3& pose,
                          OptionalJacobian<6, 6> Hself = boost::none,
                          OptionalJacobian<6, 6> Hpose = boost::none) const {
    return transformPoseTo(pose, Hself, Hpose);
  }
  /** 
  * @deprecated: this function is neither here not there. */
  Pose3 transform_to(const Pose3& pose) const;
  /// @}
#endif

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(R_);
    ar & BOOST_SERIALIZATION_NVP(t_);
  }
  /// @}

#ifdef GTSAM_USE_QUATERNIONS
  // Align if we are using Quaternions
  public:
    GTSAM_MAKE_ALIGNED_OPERATOR_NEW
#endif
};
// Pose3 class

/**
 * wedge for Pose3:
 * @param xi 6-dim twist (omega,v) where
 *  omega = 3D angular velocity
 *  v = 3D velocity
 * @return xihat, 4*4 element of Lie algebra that can be exponentiated
 */
template<>
inline Matrix wedge<Pose3>(const Vector& xi) {
  return Pose3::wedge(xi(0), xi(1), xi(2), xi(3), xi(4), xi(5));
}

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4
// deprecated: use Pose3::Align with point pairs ordered the opposite way
GTSAM_EXPORT boost::optional<Pose3> align(const std::vector<Point3Pair>& baPointPairs);
#endif

// For MATLAB wrapper
typedef std::vector<Pose3> Pose3Vector;

template <>
struct traits<Pose3> : public internal::LieGroup<Pose3> {};

template <>
struct traits<const Pose3> : public internal::LieGroup<Pose3> {};

// bearing and range traits, used in RangeFactor
template <>
struct Bearing<Pose3, Point3> : HasBearing<Pose3, Point3, Unit3> {};

template<>
struct Bearing<Pose3, Pose3> : HasBearing<Pose3, Pose3, Unit3> {};

template <typename T>
struct Range<Pose3, T> : HasRange<Pose3, T, double> {};

}  // namespace gtsam
