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
using Velocity3 = Vector3;

/**
 * Navigation state: Pose (rotation, translation) + velocity
 * Following Barrau20icra, this class belongs to the Lie group SE_2(3).
 * This group is also called "double direct isometries”.
 *
 * NOTE: While Barrau20icra follow a R,v,t order,
 * we use a R,t,v order to maintain backwards compatibility.
 */
class GTSAM_EXPORT NavState : public LieGroup<NavState, 9> {
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
  const Point3& position(OptionalJacobian<3, 9> H = {}) const;
  const Velocity3& velocity(OptionalJacobian<3, 9> H = {}) const;

  const Pose3 pose() const {
    return Pose3(attitude(), position());
  }

  /// Syntactic sugar
  const Rot3& rotation() const { return attitude(); };

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
  /// nTb = [nRb n_v, n_t; 0_1x3 1 0; 0_1x3 0 1]
  Matrix5 matrix() const;

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

  /// identity for group operation
  static NavState Identity() {
    return NavState();
  }

  /// inverse transformation with derivatives
  NavState inverse() const;

  using LieGroup<NavState, 9>::inverse;  // version with derivative

  /// compose syntactic sugar
  NavState operator*(const NavState& T) const {
    return NavState(R_ * T.R_, t_ + R_ * T.t_, v_ + R_ * T.v_);
  }

  /// @}
  /// @name Lie Group
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

  /**
   * Exponential map at identity - create a NavState from canonical coordinates
   * \f$ [R_x,R_y,R_z,T_x,T_y,T_z,V_x,V_y,V_z] \f$
   */
  static NavState Expmap(const Vector9& xi, OptionalJacobian<9, 9> Hxi = {});

  /**
   * Log map at identity - return the canonical coordinates \f$
   * [R_x,R_y,R_z,T_x,T_y,T_z,V_x,V_y,V_z] \f$ of this NavState
   */
  static Vector9 Logmap(const NavState& pose, OptionalJacobian<9, 9> Hpose = {});

  /**
   * Calculate Adjoint map, transforming a twist in this pose's (i.e, body)
   * frame to the world spatial frame.
   */
  Matrix9 AdjointMap() const;

  /**
   * Apply this NavState's AdjointMap Ad_g to a twist \f$ \xi_b \f$, i.e. a
   * body-fixed velocity, transforming it to the spatial frame
   * \f$ \xi^s = g*\xi^b*g^{-1} = Ad_g * \xi^b \f$
   * Note that H_xib = AdjointMap()
   */
  Vector9 Adjoint(const Vector9& xi_b,
                  OptionalJacobian<9, 9> H_this = {},
                  OptionalJacobian<9, 9> H_xib = {}) const;
  
  /// The dual version of Adjoint
  Vector9 AdjointTranspose(const Vector9& x,
                           OptionalJacobian<9, 9> H_this = {},
                           OptionalJacobian<9, 9> H_x = {}) const;

  /**
   * Compute the [ad(w,v)] operator as defined in [Kobilarov09siggraph], pg 11
   * but for the NavState [ad(w,v)] = [w^, zero3; v^, w^]
   */
  static Matrix9 adjointMap(const Vector9& xi);

  /**
   * Action of the adjointMap on a Lie-algebra vector y, with optional derivatives
   */
  static Vector9 adjoint(const Vector9& xi, const Vector9& y,
                         OptionalJacobian<9, 9> Hxi = {},
                         OptionalJacobian<9, 9> H_y = {});

  /**
   * The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
   */
  static Vector9 adjointTranspose(const Vector9& xi, const Vector9& y,
                                  OptionalJacobian<9, 9> Hxi = {},
                                  OptionalJacobian<9, 9> H_y = {});

  /// Derivative of Expmap
  static Matrix9 ExpmapDerivative(const Vector9& xi);

  /// Derivative of Logmap
  static Matrix9 LogmapDerivative(const NavState& xi);

  // Chart at origin, depends on compile-time flag GTSAM_POSE3_EXPMAP
  struct GTSAM_EXPORT ChartAtOrigin {
    static NavState Retract(const Vector9& xi, ChartJacobian Hxi = {});
    static Vector9 Local(const NavState& state, ChartJacobian Hstate = {});
  };

  /**
   * Compute the 6x3 bottom-left block Qs of the SE_2(3) Expmap derivative
   * matrix
   */
  static Matrix63 ComputeQforExpmapDerivative(const Vector9& xi,
                                              double nearZeroThreshold = 1e-5);

  /// @}
  /// @name Dynamics
  /// @{

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
template <>
struct traits<NavState> : public internal::LieGroup<NavState> {};

template <>
struct traits<const NavState> : public internal::LieGroup<NavState> {};

} // namespace gtsam
