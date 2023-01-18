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
 * @ingroup geometry
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
                      OptionalJacobian<6, 3> HR = {},
                      OptionalJacobian<6, 3> Ht = {});

  /**
   *  Create Pose3 by aligning two point pairs
   *  A pose aTb is estimated between pairs (a_point, b_point) such that a_point = aTb * b_point
   *  Note this allows for noise on the points but in that case the mapping will not be exact.
   */
  static std::optional<Pose3> Align(const Point3Pairs& abPointPairs);

  // Version of Pose3::Align that takes 2 matrices.
  static std::optional<Pose3> Align(const Matrix& a, const Matrix& b);

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
  static Pose3 Identity() {
    return Pose3();
  }

  /// inverse transformation with derivatives
  Pose3 inverse() const;

  /// compose syntactic sugar
  Pose3 operator*(const Pose3& T) const {
    return Pose3(R_ * T.R_, t_ + R_ * T.t_);
  }

  /**
   * Interpolate between two poses via individual rotation and translation
   * interpolation.
   *
   * The default "interpolate" method defined in Lie.h minimizes the geodesic
   * distance on the manifold, leading to a screw motion interpolation in
   * Cartesian space, which might not be what is expected.
   * In contrast, this method executes a straight line interpolation for the
   * translation, while still using interpolate (aka "slerp") for the rotational
   * component. This might be more intuitive in many applications.
   *
   * @param T End point of interpolation.
   * @param t A value in [0, 1].
   */
  Pose3 interpolateRt(const Pose3& T, double t) const {
    return Pose3(interpolate<Rot3>(R_, T.R_, t),
                 interpolate<Point3>(t_, T.t_, t));
  }

  /// @}
  /// @name Lie Group
  /// @{

  /// Exponential map at identity - create a rotation from canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$
  static Pose3 Expmap(const Vector6& xi, OptionalJacobian<6, 6> Hxi = {});

  /// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ of this rotation
  static Vector6 Logmap(const Pose3& pose, OptionalJacobian<6, 6> Hpose = {});

  /**
   * Calculate Adjoint map, transforming a twist in this pose's (i.e, body) frame to the world spatial frame
   * Ad_pose is 6*6 matrix that when applied to twist xi \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$, returns Ad_pose(xi)
   */
  Matrix6 AdjointMap() const;

  /**
   * Apply this pose's AdjointMap Ad_g to a twist \f$ \xi_b \f$, i.e. a
   * body-fixed velocity, transforming it to the spatial frame
   * \f$ \xi^s = g*\xi^b*g^{-1} = Ad_g * \xi^b \f$
   * Note that H_xib = AdjointMap()
   */
  Vector6 Adjoint(const Vector6& xi_b,
                  OptionalJacobian<6, 6> H_this = {},
                  OptionalJacobian<6, 6> H_xib = {}) const;
  
  /// The dual version of Adjoint
  Vector6 AdjointTranspose(const Vector6& x,
                           OptionalJacobian<6, 6> H_this = {},
                           OptionalJacobian<6, 6> H_x = {}) const;

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
  static Matrix6 adjointMap(const Vector6& xi);

  /**
   * Action of the adjointMap on a Lie-algebra vector y, with optional derivatives
   */
  static Vector6 adjoint(const Vector6& xi, const Vector6& y,
                         OptionalJacobian<6, 6> Hxi = {},
                         OptionalJacobian<6, 6> H_y = {});

  // temporary fix for wrappers until case issue is resolved
  static Matrix6 adjointMap_(const Vector6 &xi) { return adjointMap(xi);}
  static Vector6 adjoint_(const Vector6 &xi, const Vector6 &y) { return adjoint(xi, y);}

  /**
   * The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
   */
  static Vector6 adjointTranspose(const Vector6& xi, const Vector6& y,
                                  OptionalJacobian<6, 6> Hxi = {},
                                  OptionalJacobian<6, 6> H_y = {});

  /// Derivative of Expmap
  static Matrix6 ExpmapDerivative(const Vector6& xi);

  /// Derivative of Logmap
  static Matrix6 LogmapDerivative(const Pose3& xi);

  // Chart at origin, depends on compile-time flag GTSAM_POSE3_EXPMAP
  struct ChartAtOrigin {
    static Pose3 Retract(const Vector6& xi, ChartJacobian Hxi = {});
    static Vector6 Local(const Pose3& pose, ChartJacobian Hpose = {});
  };

  /**
  * Compute the 3x3 bottom-left block Q of SE3 Expmap right derivative matrix
  *  J_r(xi) = [J_(w) Z_3x3;
  *             Q_r   J_(w)]
  *  where J_(w) is the SO3 Expmap right derivative.
  *  (see Chirikjian11book2, pg 44, eq 10.95.
  *  The closed-form formula is identical to formula 102 in Barfoot14tro where
  *  Q_l of the SE3 Expmap left derivative matrix is given.
  */
  static Matrix3 ComputeQforExpmapDerivative(
      const Vector6& xi, double nearZeroThreshold = 1e-5);

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
      {}, OptionalJacobian<3, 3> Hpoint = {}) const;

  /**
   * @brief transform many points in Pose coordinates and transform to world.
   * @param points 3*N matrix in Pose coordinates
   * @return points in world coordinates, as 3*N Matrix
   */
  Matrix transformFrom(const Matrix& points) const;

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
      {}, OptionalJacobian<3, 3> Hpoint = {}) const;

  /**
   * @brief transform many points in world coordinates and transform to Pose.
   * @param points 3*N matrix in world coordinates
   * @return points in Pose coordinates, as 3*N Matrix
   */
  Matrix transformTo(const Matrix& points) const;

  /// @}
  /// @name Standard Interface
  /// @{

  /// get rotation
  const Rot3& rotation(OptionalJacobian<3, 6> Hself = {}) const;

  /// get translation
  const Point3& translation(OptionalJacobian<3, 6> Hself = {}) const;

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
  Pose3 transformPoseFrom(const Pose3& aTb, OptionalJacobian<6, 6> Hself = {},
                                            OptionalJacobian<6, 6> HaTb = {}) const;

  /** 
   *  Assuming self == wTa, takes a pose wTb in world coordinates 
   * and transforms it to local coordinates aTb = inv(wTa) * wTb 
   */
  Pose3 transformPoseTo(const Pose3& wTb, OptionalJacobian<6, 6> Hself = {},
                                          OptionalJacobian<6, 6> HwTb = {}) const;

  /**
   * Calculate range to a landmark
   * @param point 3D location of landmark
   * @return range (double)
   */
  double range(const Point3& point, OptionalJacobian<1, 6> Hself = {},
      OptionalJacobian<1, 3> Hpoint = {}) const;

  /**
   * Calculate range to another pose
   * @param pose Other SO(3) pose
   * @return range (double)
   */
  double range(const Pose3& pose, OptionalJacobian<1, 6> Hself = {},
      OptionalJacobian<1, 6> Hpose = {}) const;

  /**
   * Calculate bearing to a landmark
   * @param point 3D location of landmark
   * @return bearing (Unit3)
   */
  Unit3 bearing(const Point3& point, OptionalJacobian<2, 6> Hself = {},
      OptionalJacobian<2, 3> Hpoint = {}) const;

  /**
   * Calculate bearing to another pose
   * @param other 3D location and orientation of other body. The orientation
   * information is ignored.
   * @return bearing (Unit3)
   */
  Unit3 bearing(const Pose3& pose, OptionalJacobian<2, 6> Hself = {},
      OptionalJacobian<2, 6> Hpose = {}) const;

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

    /**
   * @brief Spherical Linear interpolation between *this and other
   * @param s a value between 0 and 1.5
   * @param other final point of interpolation geodesic on manifold
   */
  Pose3 slerp(double t, const Pose3& other, OptionalJacobian<6, 6> Hx = {},
                                             OptionalJacobian<6, 6> Hy = {}) const;

  /// Output stream operator
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const Pose3& p);

 private:
  /** Serialization function */
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(R_);
    ar & BOOST_SERIALIZATION_NVP(t_);
  }
#endif
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

// Convenience typedef
using Pose3Pair = std::pair<Pose3, Pose3>;
using Pose3Pairs = std::vector<std::pair<Pose3, Pose3> >;

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
