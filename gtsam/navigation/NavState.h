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
 * @author  Frank Dellaert
 * @date    July 2015
 **/

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Manifold.h>

namespace gtsam {

/// Velocity is currently typedef'd to Vector3
typedef Vector3 Velocity3;

/**
 * Navigation state: Pose (rotation, translation) + velocity
 * NOTE(frank): it does not make sense to make this a Lie group, but it is a 9D manifold
 */
class GTSAM_EXPORT NavState {
private:

  // TODO(frank):
  // - should we rename t_ to p_? if not, we should rename dP do dT
  Rot3 R_; ///< Rotation nRb, rotates points/velocities in body to points/velocities in nav
  Point3 t_; ///< position n_t, in nav frame
  Velocity3 v_; ///< velocity n_v in nav frame

public:

  enum {
    dimension = 9
  };

  typedef std::pair<Point3, Velocity3> PositionAndVelocity;

  /// @name Constructors
  /// @{

  /// Default constructor
  NavState() :
      t_(0, 0, 0), v_(Vector3::Zero()) {
  }
  /// Construct from attitude, position, velocity
  NavState(const Rot3& R, const Point3& t, const Velocity3& v) :
      R_(R), t_(t), v_(v) {
  }
  /// Construct from pose and velocity
  NavState(const Pose3& pose, const Velocity3& v) :
      R_(pose.rotation()), t_(pose.translation()), v_(v) {
  }
  /// Construct from SO(3) and R^6
  NavState(const Matrix3& R, const Vector6& tv) :
      R_(R), t_(tv.head<3>()), v_(tv.tail<3>()) {
  }
  /// Named constructor with derivatives
  static NavState Create(const Rot3& R, const Point3& t, const Velocity3& v,
      OptionalJacobian<9, 3> H1, OptionalJacobian<9, 3> H2,
      OptionalJacobian<9, 3> H3);
  /// Named constructor with derivatives
  static NavState FromPoseVelocity(const Pose3& pose, const Vector3& vel,
      OptionalJacobian<9, 6> H1, OptionalJacobian<9, 3> H2);

  /// @}
  /// @name Component Access
  /// @{

  const Rot3& attitude(OptionalJacobian<3, 9> H = {}) const;
  const Point3& position(OptionalJacobian<3, 9> H = {}) const;
  const Velocity3& velocity(OptionalJacobian<3, 9> H = {}) const;

  const Pose3 pose() const {
    return Pose3(attitude(), position());
  }

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
    return t_;
  }
  /// Return velocity as Vector3. Computation-free.
  const Vector3& v() const {
    return v_;
  }
  // Return velocity in body frame
  Velocity3 bodyVelocity(OptionalJacobian<3, 9> H = {}) const;

  /// Return matrix group representation, in MATLAB notation:
  /// nTb = [nRb 0 n_t; 0 nRb n_v; 0 0 1]
  /// With this embedding in GL(3), matrix product agrees with compose
  Matrix7 matrix() const;

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
  /// @name Manifold
  /// @{

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

  /// retract with optional derivatives
  NavState retract(const Vector9& v, //
      OptionalJacobian<9, 9> H1 = {}, OptionalJacobian<9, 9> H2 =
          {}) const;

  /// localCoordinates with optional derivatives
  Vector9 localCoordinates(const NavState& g, //
      OptionalJacobian<9, 9> H1 = {}, OptionalJacobian<9, 9> H2 =
          {}) const;

  /// @}
  /// @name Dynamics
  /// @{

  /// Integrate forward in time given angular velocity and acceleration in body frame
  /// Uses second order integration for position, returns derivatives except dt.
  NavState update(const Vector3& b_acceleration, const Vector3& b_omega,
      const double dt, OptionalJacobian<9, 9> F, OptionalJacobian<9, 3> G1,
      OptionalJacobian<9, 3> G2) const;

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
#ifdef GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(R_);
    ar & BOOST_SERIALIZATION_NVP(t_);
    ar & BOOST_SERIALIZATION_NVP(v_);
  }
#endif
  /// @}
};

// Specialize NavState traits to use a Retract/Local that agrees with IMUFactors
template<>
struct traits<NavState> : internal::Manifold<NavState> {
};

} // namespace gtsam
