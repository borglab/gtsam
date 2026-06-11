/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    NavState.h
 * @brief   Navigation state composing of attitude, position, and velocity
 * @authors Frank Dellaert, Varun Agrawal, Fan Jiang
 * @date    July 2015
 **/

#pragma once

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/ExtendedPose3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>

#if GTSAM_ENABLE_BOOST_SERIALIZATION
#include <boost/serialization/base_object.hpp>
#endif

namespace gtsam {

/// Velocity is currently typedef'd to Vector3
using Velocity3 = Vector3;

/**
 * Navigation state: Pose (rotation, translation) + velocity
 * Following Barrau20icra, this class belongs to the Lie group SE_2(3).
 * This group is also called "double direct isometries”.
 *
 * NOTE: While Barrau20icra follow a R,v,t order,
 * we use a R,t,v order to maintain backwards compatibility.
 */
class GTSAM_EXPORT NavState : public ExtendedPose3<2, NavState> {
public:
  using Base = ExtendedPose3<2, NavState>;
  using LieAlgebra = Matrix5;
  using Vector25 = Eigen::Matrix<double, 25, 1>;
  inline constexpr static auto dimension = 9;

  /// @name Constructors
  /// @{

  /// Default constructor
  NavState() : Base() {}

  NavState(const Base& other) : Base(other) {}

  /// Construct from attitude, position, velocity
  NavState(const Rot3& R, const Point3& t, const Velocity3& v)
      : Base(R, (Eigen::Matrix<double, 3, 2>() << t.x(), v.x(), t.y(), v.y(),
                 t.z(), v.z())
                    .finished()) {}

  /// Construct from pose and velocity
  NavState(const Pose3& pose, const Velocity3& v)
      : NavState(pose.rotation(), pose.translation(), v) {}

  /// Construct from SO(3) and R^6
  NavState(const Matrix3& R, const Vector6& tv)
      : NavState(Rot3(R), tv.head<3>(), tv.tail<3>()) {}

  /// Construct from Matrix5
  NavState(const Matrix5& T) : Base(T) {}

  /// Named constructor with derivatives
  static NavState Create(const Rot3& R, const Point3& t, const Velocity3& v,
                         OptionalJacobian<9, 3> H1 = {},
                         OptionalJacobian<9, 3> H2 = {},
                         OptionalJacobian<9, 3> H3 = {});

  /// Named constructor with derivatives
  static NavState FromPoseVelocity(const Pose3& pose, const Vector3& vel,
                                   OptionalJacobian<9, 6> H1 = {},
                                   OptionalJacobian<9, 3> H2 = {});

  /// @}
  /// @name Component Access
  /// @{

  const Rot3& attitude(OptionalJacobian<3, 9> H = {}) const;
  Point3 position(OptionalJacobian<3, 9> H = {}) const;
  Velocity3 velocity(OptionalJacobian<3, 9> H = {}) const;

  const Pose3 pose() const {
    return Pose3(attitude(), position());
  }

  /**
   * Calculate range to a 3D landmark.
   * @param point 3D location of landmark
   * @return range (double)
   */
  double range(const Point3& point, OptionalJacobian<1, 9> Hself = {},
               OptionalJacobian<1, 3> Hpoint = {}) const;

  /**
   * Calculate bearing to a 3D landmark.
   * @param point 3D location of landmark
   * @return bearing (Unit3)
   */
  Unit3 bearing(const Point3& point, OptionalJacobian<2, 9> Hself = {},
                OptionalJacobian<2, 3> Hpoint = {}) const;

  /// @}
  /// @name Derived quantities
  /// @{

  /// Return rotation matrix. Induces computation in quaternion mode
  Matrix3 R() const {
    return R_.matrix();
  }
  /// Return quaternion. Induces computation in matrix mode
  Quaternion quaternion() const {
    return R_.toQuaternion();
  }
  /// Return position as Vector3
  Vector3 t() const {
    return t_.col(0);
  }
  /// Return velocity as Vector3.
  Vector3 v() const {
    return velocity();
  }
  // Return velocity in body frame
  Velocity3 bodyVelocity(OptionalJacobian<3, 9> H = {}) const;

  /// @}
  /// @name Testable
  /// @{

  /// Output stream operator
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const NavState& state);

  /// print
  void print(const std::string& s = "") const;

  /// equals
  bool equals(const NavState& other, double tol = 1e-8) const;

  /// @}
  /// @name Group
  /// @{

  /// Syntactic sugar
  const Rot3& rotation(OptionalJacobian<3, 9> H = {}) const {
    return attitude(H);
  };

  // Tangent space sugar.
  // TODO(frank): move to private navstate namespace in cpp
  static Eigen::Block<Vector9, 3, 1> dR(Vector9& v) {
    return v.segment<3>(0);
  }
  static Eigen::Block<Vector9, 3, 1> dP(Vector9& v) {
    return v.segment<3>(3);
  }
  static Eigen::Block<Vector9, 3, 1> dV(Vector9& v) {
    return v.segment<3>(6);
  }
  static Eigen::Block<const Vector9, 3, 1> dR(const Vector9& v) {
    return v.segment<3>(0);
  }
  static Eigen::Block<const Vector9, 3, 1> dP(const Vector9& v) {
    return v.segment<3>(3);
  }
  static Eigen::Block<const Vector9, 3, 1> dV(const Vector9& v) {
    return v.segment<3>(6);
  }

  /**
   * Manifold retract used by optimization.
   * This intentionally uses a component-wise chart (R via Expmap, and p/v via
   * world-frame rotation of the tangent increments), not the default LieGroup
   * chart based on full Expmap/Logmap.
   */
  NavState retract(const Vector9& v, //
      OptionalJacobian<9, 9> H1 = {}, OptionalJacobian<9, 9> H2 =
          {}) const;

  /**
   * Inverse of the custom manifold chart used by retract.
   * Kept consistent with retract for optimization; Lie expmap/logmap remain
   * available separately for group operations.
   */
  Vector9 localCoordinates(const NavState& g, //
      OptionalJacobian<9, 9> H1 = {}, OptionalJacobian<9, 9> H2 =
          {}) const;

  /// @}
  /// @name Lie Group (all Lie group operations are implemented in ExtendedPose3)
  /// @{

  /// @}
  /// @name Dynamics
  /// @{

  // φ: autonomous flow where velocity acts on position for
  //   dt (R, p, v) -> p += v·dt.
  struct AutonomousFlow {
    double dt;

    // Differential at identity (right-trivialized): Φ = I with ∂p/∂v = dt·I.
    Jacobian dIdentity() const {
      Jacobian Phi = I_9x9;
      Phi.template block<3, 3>(3, 6) = I_3x3 * dt;
      return Phi;
    }

    // Apply φ(x) by p += v·dt
    NavState operator()(const NavState& X) const {
      return {X.attitude(), X.position() + X.velocity() * dt, X.velocity()};
    }
  };

/// Integrate forward in time given angular velocity and acceleration in body frame
  /// Uses second order integration for position, returns derivatives except dt.
  NavState update(const Vector3& b_acceleration, const Vector3& b_omega,
                  const double dt, OptionalJacobian<9, 9> F = {},
                  OptionalJacobian<9, 3> G1 = {},
                  OptionalJacobian<9, 3> G2 = {}) const;

  /// Compute tangent space contribution due to Coriolis forces
  Vector9 coriolis(double dt, const Vector3& omega, bool secondOrder = false,
      OptionalJacobian<9, 9> H = {}) const;

  /// Correct preintegrated tangent vector with our velocity and rotated gravity,
  /// taking into account Coriolis forces if omegaCoriolis is given.
  Vector9 correctPIM(const Vector9& pim, double dt, const Vector3& n_gravity,
      const std::optional<Vector3>& omegaCoriolis, bool use2ndOrderCoriolis =
          false, OptionalJacobian<9, 9> H1 = {},
      OptionalJacobian<9, 9> H2 = {}) const;

  /// @}

private:
  /// @{
  /// serialization
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
  /// @}
};

// Specialize NavState traits to use a Retract/Local that agrees with IMUFactors
template <>
struct traits<NavState> : public internal::MatrixLieGroup<NavState, 5> {};

template <>
struct traits<const NavState> : public internal::MatrixLieGroup<NavState, 5> {};

// bearing and range traits, used in RangeFactor and BearingFactor
template <>
struct Bearing<NavState, Point3> : HasBearing<NavState, Point3, Unit3> {};

template <>
struct Range<NavState, Point3> : HasRange<NavState, Point3, double> {};

} // namespace gtsam
