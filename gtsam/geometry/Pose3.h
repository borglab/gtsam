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
  static std::optional<Pose3> Align(ConstMatrixView a, ConstMatrixView b);

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
  Matrix transformFrom(ConstMatrixView points) const;

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
  Matrix transformTo(ConstMatrixView points) const;

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

/**
 * Add the Pose3-specific operations used by the generic QCQP conversion code.
 *
 * `template <> struct traits<Pose3>` specializes GTSAM's generic `traits<T>`
 * interface for T=Pose3. Its static methods are called directly through the
 * traits type. The member-template parameter D is the QCQP variable's column
 * count, and `if constexpr (D == 1)` selects this exact homogenized vector
 * formulation at compile time.
 *
 * The lift discards the fixed last row [0, 0, 0, 1] of the Pose3 matrix and
 * stores the first three entries of each column consecutively:
 *
 *   x = [1, r00, r10, r20, r01, r11, r21,
 *           r02, r12, r22, tx, ty, tz]'.
 *
 * Eigen's `segment<3>(start)` selects three consecutive entries of x, while
 * `T.col(column).head<3>()` selects the retained part of one matrix column.
 * QcqpConstraints returns the symmetric matrices A and scalars b for the ten
 * equations x' A x = b. Their translation rows and columns are zero, so these
 * equations constrain only the embedded SO(3) rotation.
 */
template <>
struct traits<Pose3> : public internal::MatrixLieGroup<Pose3, 4> {
  /// Dimension of the D=1 homogenized QCQP vector.
  inline constexpr static int QcqpVectorDim = 13;

  /**
   * Return the D=1 homogenized QCQP variable
   * x = [1, vec(R), tx, ty, tz] in column-major order.
   */
  template <int D = 1>
  static Matrix QcqpValue(const Pose3& value) {
    if constexpr (D == 1) {
      const Matrix4 T = value.matrix();
      Eigen::Matrix<double, 13, 1> X;
      X(0, 0) = 1.0;
      X.segment<3>(1) = T.col(0).head<3>();
      X.segment<3>(4) = T.col(1).head<3>();
      X.segment<3>(7) = T.col(2).head<3>();
      X.segment<3>(10) = T.col(3).head<3>();
      return X;
    } else {
      throw std::invalid_argument(
          "traits<Pose3>::QcqpValue only supports D=1.");
    }
  }

  /**
   * Return the ten D=1 lifted SE(3) manifold constraints A, b such that
   * trace(x' A x) = b.
   */
  template <int D = 1>
  static std::vector<std::pair<Matrix, double>> QcqpConstraints() {
    if constexpr (D == 1) {
      std::vector<std::pair<Matrix, double>> constraints;
      constraints.reserve(10);

      Matrix A = Matrix::Zero(13, 13);

      // Homogenization.
      A(0, 0) = 1.0;
      constraints.emplace_back(A, 1.0);

      // cross(R.col(1), R.col(2)) = x(0) * R.col(0).
      A.setZero();
      A(5, 9) = 0.5;
      A(9, 5) = 0.5;
      A(6, 8) = -0.5;
      A(8, 6) = -0.5;
      A(0, 1) = -0.5;
      A(1, 0) = -0.5;
      constraints.emplace_back(A, 0.0);

      A.setZero();
      A(6, 7) = 0.5;
      A(7, 6) = 0.5;
      A(4, 9) = -0.5;
      A(9, 4) = -0.5;
      A(0, 2) = -0.5;
      A(2, 0) = -0.5;
      constraints.emplace_back(A, 0.0);

      A.setZero();
      A(4, 8) = 0.5;
      A(8, 4) = 0.5;
      A(5, 7) = -0.5;
      A(7, 5) = -0.5;
      A(0, 3) = -0.5;
      A(3, 0) = -0.5;
      constraints.emplace_back(A, 0.0);

      // RR^T = I.
      A.setZero();
      A(1, 1) = 1.0;
      A(4, 4) = 1.0;
      A(7, 7) = 1.0;
      constraints.emplace_back(A, 1.0);

      A.setZero();
      A(1, 2) = 0.5;
      A(2, 1) = 0.5;
      A(4, 5) = 0.5;
      A(5, 4) = 0.5;
      A(7, 8) = 0.5;
      A(8, 7) = 0.5;
      constraints.emplace_back(A, 0.0);

      A.setZero();
      A(1, 3) = 0.5;
      A(3, 1) = 0.5;
      A(4, 6) = 0.5;
      A(6, 4) = 0.5;
      A(7, 9) = 0.5;
      A(9, 7) = 0.5;
      constraints.emplace_back(A, 0.0);

      A.setZero();
      A(2, 2) = 1.0;
      A(5, 5) = 1.0;
      A(8, 8) = 1.0;
      constraints.emplace_back(A, 1.0);

      A.setZero();
      A(2, 3) = 0.5;
      A(3, 2) = 0.5;
      A(5, 6) = 0.5;
      A(6, 5) = 0.5;
      A(8, 9) = 0.5;
      A(9, 8) = 0.5;
      constraints.emplace_back(A, 0.0);

      A.setZero();
      A(3, 3) = 1.0;
      A(6, 6) = 1.0;
      A(9, 9) = 1.0;
      constraints.emplace_back(A, 1.0);

      return constraints;
    } else {
      throw std::invalid_argument(
          "traits<Pose3>::QcqpConstraints only supports D=1.");
    }
  }

  /** Project a D=1 homogenized QCQP vector back to Pose3. */
  template <int D>
  static Pose3 FromQcqpValue(const Matrix& X) {
    if constexpr (D == 1) {
      if (X.rows() != QcqpVectorDim || X.cols() != 1 ||
          std::abs(X(0, 0)) < 1e-9) {
        throw std::invalid_argument(
            "traits<Pose3>::FromQcqpValue requires a 13-by-1 vector with a "
            "nonzero homogenization entry.");
      }
      const Vector x = X.col(0) / X(0, 0);
      Matrix3 R;
      R.col(0) = x.segment<3>(1);
      R.col(1) = x.segment<3>(4);
      R.col(2) = x.segment<3>(7);
      return Pose3(Rot3::ClosestTo(R), x.segment<3>(10));
    } else {
      throw std::invalid_argument(
          "traits<Pose3>::FromQcqpValue only supports D=1.");
    }
  }
};

template <>
struct traits<const Pose3> : public traits<Pose3> {};

// bearing and range traits, used in RangeFactor
template <>
struct Bearing<Pose3, Point3> : HasBearing<Pose3, Point3, Unit3> {};

template<>
struct Bearing<Pose3, Pose3> : HasBearing<Pose3, Pose3, Unit3> {};

template <typename T>
struct Range<Pose3, T> : HasRange<Pose3, T, double> {};

}  // namespace gtsam
