/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *@file  Pose3.h
 * @brief 3D Pose manifold SO(3) x R^3 and group SE(3)
 */

// \callgraph
#pragma once

#include <gtsam/config.h>

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Lie.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/base_object.hpp>
#endif

namespace gtsam {

class Pose2;
// forward declare

/**
 * A 3D pose (R,t) : (Rot3,Point3)
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Pose3: public ExtendedPose3<1, Pose3> {
public:
  using Base = ExtendedPose3<1, Pose3>;

  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Translation;
  inline constexpr static auto dimension = 6;

public:
  using Vector16 = Eigen::Matrix<double, 16, 1>;
  using Base::operator*;

  /// @name Standard Constructors
  /// @{

  /** Default constructor is origin */
  Pose3() : Base() {}

  /** Copy constructor */
  Pose3(const Pose3& pose) = default;

  Pose3& operator=(const Pose3& other) = default;

  Pose3(const Base& other) : Base(other) {}

  /** Construct from R,t */
  Pose3(const Rot3& R, const Point3& t)
      : Base(R, Vector3(t.x(), t.y(), t.z())) {}

  /** Construct from Pose2 */
  explicit Pose3(const Pose2& pose2);

  /** Constructor from 4*4 matrix */
  Pose3(const Matrix &T) : Base(Matrix4(T)) {}

  /// Named constructor with derivatives
  static Pose3 Create(const Rot3& R, const Point3& t,
                      OptionalJacobian<6, 3> HR = {},
                      OptionalJacobian<6, 3> Ht = {});

  /** Construct from Pose2 in the xy plane, with derivative. */
  static Pose3 FromPose2(const Pose2& p, OptionalJacobian<6,3> H = {});

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
  Pose3 interpolateRt(const Pose3& T, double t,
                      OptionalJacobian<6, 6> Hself = {},
                      OptionalJacobian<6, 6> Harg = {},
                      OptionalJacobian<6, 1> Ht = {}) const;

  /// Compose syntactic sugar.
  Pose3 operator*(const Pose3& T) const {
    return Pose3(R_ * T.R_, t_ + R_ * T.t_);
  }

  /// @}
  /// @name Lie Group
  /// @{

  using LieAlgebra = Matrix4;

  /// Exponential map at identity.
  static Pose3 Expmap(const Vector6& xi, OptionalJacobian<6, 6> Hxi = {});

  // temporary fix for wrappers until case issue is resolved
  static Matrix6 adjointMap_(const Vector6 &xi) { return adjointMap(xi);}
  static Vector6 adjoint_(const Vector6 &xi, const Vector6 &y) { return adjoint(xi, y);}

  // Chart at origin, depends on compile-time flag GTSAM_POSE3_EXPMAP
  struct GTSAM_EXPORT ChartAtOrigin {
    static Pose3 Retract(const Vector6& xi, ChartJacobian Hxi = {});
    static Vector6 Local(const Pose3& pose, ChartJacobian Hpose = {});
  };

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

  /// get translation
  const Point3& translation(OptionalJacobian<3, 6> Hself = {}) const;

  /// get x
  double x() const {
    return translation().x();
  }

  /// get y
  double y() const {
    return translation().y();
  }

  /// get z
  double z() const {
    return translation().z();
  }

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
    return {3, 5};
  }

  /**
   * Return the start and end indices (inclusive) of the rotation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  static std::pair<size_t, size_t> rotationInterval() {
    return {0, 2};
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

  /// @}
  /// @name deprecated
  /// @{

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  /// @deprecated: use Hat
  static inline LieAlgebra wedge(double wx, double wy, double wz, double vx,
                                 double vy, double vz) {
    return Hat((TangentVector() << wx, wy, wz, vx, vy, vz).finished());
  }
#endif
  /// @}

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  /** Serialization function */
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

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
/// @deprecated: use T::Hat
template<>
inline Matrix wedge<Pose3>(const Vector& xi) {
  // NOTE(chris): Need eval() as workaround for Apple clang + avx2.
  return Matrix(Pose3::Hat(xi)).eval();
}
#endif

// Convenience typedef
using Pose3Pair = std::pair<Pose3, Pose3>;
using Pose3Pairs = std::vector<std::pair<Pose3, Pose3> >;

// For MATLAB wrapper
typedef std::vector<Pose3> Pose3Vector;

template <>
struct traits<Pose3> : public internal::MatrixLieGroup<Pose3, 4> {};

template <>
struct traits<const Pose3> : public internal::MatrixLieGroup<Pose3, 4> {};

// bearing and range traits, used in RangeFactor
template <>
struct Bearing<Pose3, Point3> : HasBearing<Pose3, Point3, Unit3> {};

template<>
struct Bearing<Pose3, Pose3> : HasBearing<Pose3, Pose3, Unit3> {};

template <typename T>
struct Range<Pose3, T> : HasRange<Pose3, T, double> {};

}  // namespace gtsam
