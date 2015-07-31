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
#include <gtsam/base/ProductLieGroup.h>

namespace gtsam {

/// Velocity is currently typedef'd to Vector3
typedef Vector3 Velocity3;

/**
 * Navigation state: Pose (rotation, translation) + velocity
 * Implements semi-direct Lie group product of SO(3) and R^6, where R^6 is position/velocity
 */
class NavState: public LieGroup<NavState, 9> {
private:

  // TODO(frank):
  // - should we rename t_ to p_? if not, we should rename dP do dT
  Rot3 R_; ///< Rotation nRb, rotates points/velocities in body to points/velocities in nav
  Point3 t_; ///< position n_t, in nav frame
  Velocity3 v_; ///< velocity n_v in nav frame

public:

  typedef std::pair<Point3, Velocity3> PositionAndVelocity;

  /// @name Constructors
  /// @{

  /// Default constructor
  NavState() :
      v_(Vector3::Zero()) {
  }
  /// Construct from attitude, position, velocity
  NavState(const Rot3& R, const Point3& t, const Velocity3& v) :
      R_(R), t_(t), v_(v) {
  }
  /// Construct from pose and velocity
  NavState(const Pose3& pose, const Velocity3& v) :
      R_(pose.rotation()), t_(pose.translation()), v_(v) {
  }
  /// Construct from Matrix group representation (no checking)
  NavState(const Matrix7& T) :
      R_(T.block<3, 3>(0, 0)), t_(T.block<3, 1>(0, 6)), v_(T.block<3, 1>(3, 6)) {
  }
  /// Construct from SO(3) and R^6
  NavState(const Matrix3& R, const Vector9 tv) :
      R_(R), t_(tv.head<3>()), v_(tv.tail<3>()) {
  }
  /// Named constructor with derivatives
  static NavState FromPoseVelocity(const Pose3& pose, const Vector3& vel,
      OptionalJacobian<9, 6> H1, OptionalJacobian<9, 3> H2);

  /// @}
  /// @name Component Access
  /// @{

  inline const Rot3& attitude() const {
    return R_;
  }
  inline const Point3& position() const {
    return t_;
  }
  inline const Velocity3& velocity() const {
    return v_;
  }
  const Rot3& attitude(OptionalJacobian<3, 9> H) const;
  const Point3& position(OptionalJacobian<3, 9> H) const;
  const Velocity3& velocity(OptionalJacobian<3, 9> H) const;

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
    return t_.vector();
  }
  /// Return velocity as Vector3. Computation-free.
  const Vector3& v() const {
    return v_;
  }
  // Return velocity in body frame
  Velocity3 bodyVelocity(OptionalJacobian<3, 9> H = boost::none) const;

  /// Return matrix group representation, in MATLAB notation:
  /// nTb = [nRb 0 n_t; 0 nRb n_v; 0 0 1]
  /// With this embedding in GL(3), matrix product agrees with compose
  Matrix7 matrix() const;

  /// @}
  /// @name Testable
  /// @{

  /// print
  void print(const std::string& s = "") const;

  /// equals
  bool equals(const NavState& other, double tol = 1e-8) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  static NavState identity() {
    return NavState();
  }

  /// inverse transformation with derivatives
  NavState inverse() const;

  /// Group compose is the semi-direct product as specified below:
  /// nTc = nTb * bTc = (nRb * bRc, nRb * b_t + n_t, nRb * b_v + n_v)
  NavState operator*(const NavState& bTc) const;

  /// Native group action is on position/velocity pairs *in the body frame* as follows:
  /// nTb * (b_t,b_v) = (nRb * b_t + n_t, nRb * b_v + n_v)
  PositionAndVelocity operator*(const PositionAndVelocity& b_tv) const;

  /// Act on position alone, n_t = nRb * b_t + n_t
  Point3 operator*(const Point3& b_t) const;

  /// Act on velocity alone, n_v = nRb * b_v + n_v
  Velocity3 operator*(const Velocity3& b_v) const;

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

  // Chart at origin, constructs components separately (as opposed to Expmap)
  struct ChartAtOrigin {
    static NavState Retract(const Vector9& xi, //
        OptionalJacobian<9, 9> H = boost::none);
    static Vector9 Local(const NavState& x, //
        OptionalJacobian<9, 9> H = boost::none);
  };

  /// @}
  /// @name Lie Group
  /// @{

  /// Exponential map at identity - create a NavState from canonical coordinates
  static NavState Expmap(const Vector9& xi, //
      OptionalJacobian<9, 9> H = boost::none);

  /// Log map at identity - return the canonical coordinates for this NavState
  static Vector9 Logmap(const NavState& p, //
      OptionalJacobian<9, 9> H = boost::none);

  /// Calculate Adjoint map, a 9x9 matrix that takes a tangent vector in the body frame, and transforms
  /// it to a tangent vector at identity, such that Exmap(AdjointMap()*xi) = (*this) * Exmpap(xi);
  Matrix9 AdjointMap() const;

  /// wedge creates Lie algebra element from tangent vector
  static Matrix7 wedge(const Vector9& xi);

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
      OptionalJacobian<9, 9> H = boost::none) const;

  /// Correct preintegrated tangent vector with our velocity and rotated gravity,
  /// taking into account Coriolis forces if omegaCoriolis is given.
  Vector9 correctPIM(const Vector9& pim, double dt, const Vector3& n_gravity,
      const boost::optional<Vector3>& omegaCoriolis, bool use2ndOrderCoriolis =
          false, OptionalJacobian<9, 9> H1 = boost::none,
      OptionalJacobian<9, 9> H2 = boost::none) const;

private:
  /// @{
  /// serialization
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(R_);
    ar & BOOST_SERIALIZATION_NVP(t_);
    ar & BOOST_SERIALIZATION_NVP(v_);
  }
  /// @}
};

// Specialize NavState traits to use a Retract/Local that agrees with IMUFactors
template<>
struct traits<NavState> : Testable<NavState>, internal::LieGroupTraits<NavState> {
};

// Partial specialization of wedge
// TODO: deprecate, make part of traits
template<>
inline Matrix wedge<NavState>(const Vector& xi) {
  return NavState::wedge(xi);
}

} // namespace gtsam
