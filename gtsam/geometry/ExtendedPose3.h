/* ----------------------------------------------------------------------------

* GTSAM Copyright 2010, Georgia Tech Research Corporation,
* Atlanta, Georgia 30332-0415
* All Rights Reserved
* Authors: Frank Dellaert, et al. (see THANKS for the full author list)

* See LICENSE for the license information

* -------------------------------------------------------------------------- */

/**
 * @file  ExtendedPose3.h
 * @brief 3D Extended Pose
 * @author Martin Brossard
 */

// \callgraph
#pragma once

#include <gtsam/config.h>

#include <gtsam/base/Lie.h>
#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

namespace gtsam {

/**
 * A 3D extended pose (R,v,p) : (Rot3,Point3,Point3)
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT ExtendedPose3 : public LieGroup<ExtendedPose3, 9> {
 public:
  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Velocity;
  typedef Point3 Position;
  typedef Position Translation;

 private:
  Rot3 R_;    ///< Rotation gRp, between global and pose frame
  Point3 v_;  ///< Velocity gVp, from global origin to pose frame origin
  Point3 p_;  ///< Position gPp, from global origin to pose frame origin

 public:
  /// @name Standard Constructors
  /// @{

  /** Default constructor is origin */
  ExtendedPose3() : R_(traits<Rot3>::Identity()), v_(traits<Point3>::Identity()), p_(traits<Point3>::Identity()) {}

  /** Copy constructor */
  ExtendedPose3(const ExtendedPose3& pose) : R_(pose.R_), v_(pose.v_), p_(pose.p_) {}

  /** Construct from R,v,p */
  ExtendedPose3(const Rot3& R, const Point3& v, const Point3& p) : R_(R), v_(v), p_(p) {}

  // explicit Pose3(const Pose2& pose2);

  /** Constructor from 5*5 matrix */
  explicit ExtendedPose3(const Matrix& T)
      : R_(T(0, 0), T(0, 1), T(0, 2), T(1, 0), T(1, 1), T(1, 2), T(2, 0), T(2, 1), T(2, 2)),
        v_(T(0, 3), T(1, 3), T(2, 3)),
        p_(T(0, 4), T(1, 4), T(2, 4)) {}

  /// Named constructor with derivatives
  static ExtendedPose3 Create(const Rot3& R, const Point3& v, const Point3& p, OptionalJacobian<9, 3> HR = {},
                              OptionalJacobian<9, 3> Hv = {}, OptionalJacobian<9, 3> Hp = {});

  /**
   *  Create Pose3 by aligning two point pairs
   *  A pose aTb is estimated between pairs (a_point, b_point) such that a_point = aTb * b_point
   *  Note this allows for noise on the points but in that case the mapping will not be exact.
   */
  static std::optional<ExtendedPose3> Align(const std::vector<Point3Pair>& abPointPairs);

  /// @}
  /// @name Testable
  /// @{

  /// print with optional string
  void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const ExtendedPose3& pose, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static ExtendedPose3 Identity() { return ExtendedPose3(); }

  /// inverse transformation with derivatives
  ExtendedPose3 inverse() const;

  /// compose syntactic sugar
  ExtendedPose3 operator*(const ExtendedPose3& T) const {
    return ExtendedPose3(R_ * T.R_, v_ + R_ * T.v_, p_ + R_ * T.p_);
  }

  /// @}
  /// @name Lie Group
  /// @{

  /// Exponential map at identity - create a rotation from canonical coordinates \f$
  /// [R_x,R_y,R_z,V_x,V_y,V_z,P_x,P_y,P_z] \f$
  static ExtendedPose3 Expmap(const Vector9& xi, OptionalJacobian<9, 9> Hxi = {});

  /// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z,V_x,V_y,V_z,P_x,P_y,P_z] \f$ of this
  /// rotation
  static Vector9 Logmap(const ExtendedPose3& pose, OptionalJacobian<9, 9> Hpose = {});

  /**
   * Calculate Adjoint map, transforming a twist in the this pose's (i.e, body) frame to the world spatial frame
   * Ad_pose is 9*9 matrix that when applied to twist xi \f$ [R_x,R_y,R_z,V_x,V_y,V_z,P_x,P_y,P_z] \f$, returns
   * Ad_pose(xi)
   */
  Matrix9 AdjointMap() const;  /// FIXME Not tested - marked as incorrect

  /**
   * Apply this pose's AdjointMap Ad_g to a twist \f$ \xi_b \f$, i.e. a body-fixed velocity, transforming it to the
   * spatial frame \f$ \xi^s = g*\xi^b*g^{-1} = Ad_g * \xi^b \f$
   */
  Vector9 Adjoint(const Vector9& xi_b) const { return AdjointMap() * xi_b; }  /// FIXME Not tested - marked as incorrect

  /**
   * Compute the ad operator
   */
  static Matrix9 adjointMap(const Vector9& xi);

  /**
   * Action of the adjointMap on a Lie-algebra vector y, with optional derivatives
   */
  static Vector9 adjoint(const Vector9& xi, const Vector9& y, OptionalJacobian<9, 9> Hxi = {});

  // temporary fix for wrappers until case issue is resolved
  static Matrix9 adjointMap_(const Vector9& xi) { return adjointMap(xi); }
  static Vector9 adjoint_(const Vector9& xi, const Vector9& y) { return adjoint(xi, y); }

  /**
   * The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
   */
  static Vector9 adjointTranspose(const Vector9& xi, const Vector9& y, OptionalJacobian<9, 9> Hxi = {});

  /// Derivative of Expmap
  static Matrix9 ExpmapDerivative(const Vector9& xi);

  /// Derivative of Logmap
  static Matrix9 LogmapDerivative(const ExtendedPose3& xi);

  // Chart at origin, depends on compile-time flag GTSAM_POSE3_EXPMAP
  struct ChartAtOrigin {
    static ExtendedPose3 Retract(const Vector9& xi, ChartJacobian Hxi = {});
    static Vector9 Local(const ExtendedPose3& pose, ChartJacobian Hpose = {});
  };

  Vector9 boxminus(const ExtendedPose3& g) const;

  using LieGroup<ExtendedPose3, 9>::inverse;  // version with derivative

  /**
   * wedge for ExtendedPose3:
   * @param xi 9-dim twist (omega,nu,rho)
   * @return 5*5 element of Lie algebra
   */
  static Matrix wedge(double phix, double phiy, double phiz, double nux, double nuy, double nuz, double rhox,
                      double rhoy, double rhoz) {
    return (Matrix(5, 5) << 0., -phiz, phiy, nux, rhox, phiz, 0., -phix, nuy, rhoy, -phiy, phix, 0., nuz, rhoz, 0., 0.,
            0., 0., 0., 0., 0., 0., 0., 0.)
        .finished();
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
  Point3 transformFrom(const Point3& point, OptionalJacobian<3, 9> Hself = {},
                       OptionalJacobian<3, 3> Hpoint = {}) const;

  /** syntactic sugar for transformFrom */
  inline Point3 operator*(const Point3& point) const { return transformFrom(point); }

  /**
   * @brief takes point in world coordinates and transforms it to Pose coordinates
   * @param point point in world coordinates
   * @param Hself optional 3*6 Jacobian wrpt this pose
   * @param Hpoint optional 3*3 Jacobian wrpt point
   * @return point in Pose coordinates
   */
  Point3 transformTo(const Point3& point, OptionalJacobian<3, 9> Hself = {}, OptionalJacobian<3, 3> Hpoint = {}) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// get rotation
  const Rot3& rotation(OptionalJacobian<3, 9> Hself = {}) const;

  /// get translation
  const Point3& velocity(OptionalJacobian<3, 9> Hself = {}) const;

  /// get position
  const Point3& position(OptionalJacobian<3, 9> Hself = {}) const;

  const Point3& translation(OptionalJacobian<3, 9> Hself = {}) const;

  /// get x
  double x() const { return p_.x(); }

  /// get y
  double y() const { return p_.y(); }

  /// get z
  double z() const { return p_.z(); }

  /** convert to 5*5 matrix */
  Matrix5 matrix() const;

  /**
   * Assuming self == wTa, takes a pose aTb in local coordinates
   * and transforms it to world coordinates wTb = wTa * aTb.
   * This is identical to compose.
   */
  ExtendedPose3 transformPoseFrom(const ExtendedPose3& aTb, OptionalJacobian<9, 9> Hself = {},
                                  OptionalJacobian<9, 9> HaTb = {}) const;

  /**
   *  Assuming self == wTa, takes a pose wTb in world coordinates
   * and transforms it to local coordinates aTb = inv(wTa) * wTb
   */
  ExtendedPose3 transformPoseTo(const ExtendedPose3& wTb, OptionalJacobian<9, 9> Hself = {},
                                OptionalJacobian<9, 9> HwTb = {}) const;

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @return range (double)
   */
  double range(const Point3& point, OptionalJacobian<1, 9> Hself = {}, OptionalJacobian<1, 3> Hpoint = {}) const;

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @return range (double)
   */
  double range(const ExtendedPose3& pose, OptionalJacobian<1, 9> Hself = {}, OptionalJacobian<1, 9> Hpose = {}) const;

  /**
   * Calculate bearing to a landmark
   * @param point 3D location of landmark
   * @return bearing (Unit3)
   */
  Unit3 bearing(const Point3& point, OptionalJacobian<2, 9> Hself = {}, OptionalJacobian<2, 3> Hpoint = {}) const;

  /**
   * Calculate bearing to another pose
   * @param other 3D location and orientation of other body. The orientation
   * information is ignored.
   * @return bearing (Unit3)
   */
  Unit3 bearing(const ExtendedPose3& pose, OptionalJacobian<2, 9> Hself = {}, OptionalJacobian<2, 9> Hpose = {}) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Return the start and end indices (inclusive) of the translation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  inline static std::pair<size_t, size_t> velocityInterval() { return std::make_pair(3, 5); }

  /**
   * Return the start and end indices (inclusive) of the translation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  inline static std::pair<size_t, size_t> positionInterval() { return std::make_pair(6, 8); }
  inline static std::pair<size_t, size_t> translationInterval() { return std::make_pair(6, 8); }

  /**
   * Return the start and end indices (inclusive) of the rotation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  static std::pair<size_t, size_t> rotationInterval() { return std::make_pair(0, 2); }

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const ExtendedPose3& p);

 private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(R_);
    ar& BOOST_SERIALIZATION_NVP(v_);
    ar& BOOST_SERIALIZATION_NVP(p_);
  }
  /// @}

 public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
};  // ExtendedPose3 class

/**
 * wedge for ExtendedPose3:
 * @param xi 9-dim twist (omega,nu,rho)
 * @return 5*5 element of Lie algebra
 */
template <>
inline Matrix wedge<ExtendedPose3>(const Vector& xi) {
  return ExtendedPose3::wedge(xi(0), xi(1), xi(2), xi(3), xi(4), xi(5), xi(6), xi(7), xi(8));
}

template <>
struct traits<ExtendedPose3> : public internal::LieGroup<ExtendedPose3> {};

template <>
struct traits<const ExtendedPose3> : public internal::LieGroup<ExtendedPose3> {};

// bearing and range traits, used in RangeFactor
template <>
struct Bearing<ExtendedPose3, Point3> : HasBearing<ExtendedPose3, Point3, Unit3> {};

template <>
struct Bearing<ExtendedPose3, ExtendedPose3> : HasBearing<ExtendedPose3, ExtendedPose3, Unit3> {};

template <typename T>
struct Range<ExtendedPose3, T> : HasRange<ExtendedPose3, T, double> {};
}  // namespace gtsam
