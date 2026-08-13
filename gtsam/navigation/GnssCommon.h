/**
 *  @file   GnssCommon.h
 *  @brief  Shared constants and utilities for GNSS factors.
 *  @date   April 14, 2026
 **/
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/dllexport.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <optional>

namespace gtsam {

/**
 * Base class storing common members for GNSS measurement factors.
 *
 * Shared by pseudorange and carrier phase factors.  The measurement field is
 * in meters (i.e. for carrier phase factors it is already multiplied by the
 * wavelength, `lambda * phi_cycles`).
 */
struct GnssMeasurementBase {
  /// Measurement in meters (pseudorange, or carrier phase * wavelength).
  double measurement_;
  /// Satellite position in WGS84 ECEF meters.
  Point3 satPos_;
  /// Satellite clock bias in seconds.
  double satClkBias_;
};

namespace gnss {

/// Speed of light in a vacuum (m/s).
constexpr double C_LIGHT = 299792458.0;

/// WGS-84 Earth rotation rate (rad/s).
constexpr double OMGE = 7.2921151467e-5;

/**
 * Geometric distance with Sagnac correction.
 *
 * Computes the Euclidean distance between a satellite and a receiver in ECEF,
 * plus the first-order Sagnac correction for the rotating Earth.  When the
 * satellite and receiver positions coincide (within machine epsilon) the unit
 * vector is set to zero and the optional Jacobian is set to zero, which avoids
 * NaNs from dividing by zero.
 *
 * @param[in]  sat   Satellite ECEF position (m).
 * @param[in]  rcv   Receiver ECEF position (m).
 * @param[out] e     Unit vector from receiver to satellite.
 * @param[out] H_rcv Optional Jacobian of the returned range w.r.t. rcv (1x3).
 *                   Includes the contribution from the Sagnac term.
 * @return     Geometric range with Sagnac correction (m).
 */
GTSAM_EXPORT double geodist(const Point3& sat, const Point3& rcv, Point3& e,
                            OptionalJacobian<1, 3> H_rcv = {});

/**
 * Shared geometry data for double-difference factors.
 *
 * Holds the four undifferenced observations (rover/base, ref/target satellite),
 * the corresponding satellite ECEF positions at rover and base observation
 * times, and the known base station ECEF position.  The struct owns the
 * computation of the observed double-difference and of the geometric DD model
 * range (with Sagnac-corrected geodist) plus its Jacobian w.r.t. the rover or
 * antenna ECEF position.
 *
 * Used by both the pseudorange and carrier-phase DD factors so the DD
 * geometry/Jacobian implementation is shared.
 */
struct GTSAM_EXPORT DoubleDifferenceData {
  double rovRef = 0;      ///< Rover observation for ref satellite [m].
  double baseRef = 0;     ///< Base  observation for ref satellite [m].
  double rovTarget = 0;   ///< Rover observation for target satellite [m].
  double baseTarget = 0;  ///< Base  observation for target satellite [m].
  Point3 satRefRov{0, 0, 0};      ///< Ref satellite ECEF at rover time [m].
  Point3 satTargetRov{0, 0, 0};   ///< Target satellite ECEF at rover time [m].
  Point3 satRefBase{0, 0, 0};     ///< Ref satellite ECEF at base time [m].
  Point3 satTargetBase{0, 0, 0};  ///< Target satellite ECEF at base time [m].
  Point3 basePos{0, 0, 0};        ///< Base station ECEF position [m].

  /// Observed DD: (rovRef - baseRef) - (rovTarget - baseTarget) [m].
  double observed() const {
    return (rovRef - baseRef) - (rovTarget - baseTarget);
  }

  /**
   * Geometric DD model range and its Jacobian w.r.t. the rover/antenna ECEF
   * position, using Sagnac-corrected geodist for both rover and base sides:
   *   model = (geodist(satRefRov, rcv)    - geodist(satRefBase, basePos))
   *         - (geodist(satTargetRov, rcv) - geodist(satTargetBase, basePos))
   * Base-side ranges have no derivative because basePos is fixed.
   *
   * @param[in]  rcv    Rover (or antenna) ECEF position [m].
   * @param[out] H_rcv  Optional 1x3 Jacobian of the model w.r.t. rcv.
   * @return     Geometric DD model range [m].
   */
  double model(const Point3& rcv, OptionalJacobian<1, 3> H_rcv = {}) const;

  bool equals(const DoubleDifferenceData& other, double tol) const;
};

/**
 * Lever-arm helper for GNSS factors that key on a body Pose3.
 *
 * Owns the lever arm (body-frame translation from the body origin to the GNSS
 * antenna) and an optional ecef_T_nav transform.  Centralizes the two
 * operations that every "Arm" GNSS factor needs:
 *
 *   1. Map a body Pose3 (optionally expressed in a local nav frame) plus a
 *      lever arm to an ECEF antenna position.
 *   2. Push a 1x3 Jacobian w.r.t. the antenna position back through the lever
 *      arm and (optional) ecef_T_nav compose to produce the 1x6 Jacobian
 *      w.r.t. the pose tangent.
 */
struct GTSAM_EXPORT LeverArm {
  Point3 b{0, 0, 0};                  ///< Lever arm in body frame [m].
  std::optional<Pose3> ecef_T_nav;    ///< Optional ECEF-from-nav transform.

  LeverArm() = default;
  explicit LeverArm(const Point3& leverArm) : b(leverArm) {}
  LeverArm(const Point3& leverArm, const Pose3& nav)
      : b(leverArm), ecef_T_nav(nav) {}

  /// Intermediate quantities cached by antennaPosition() so antennaPoseJacobian
  /// can compute the pose Jacobian without recomputing the rotation/compose.
  struct PoseFrame {
    Matrix3 ecef_R_body;  ///< Rotation of the body in ECEF.
    Matrix66 H_compose;   ///< d(ecef_T_body)/d(pose) (only used if has_nav).
    bool has_nav = false;
  };

  /**
   * Compute the antenna ECEF position for `pose`.
   *
   * If `frame` is non-null it is filled with the chain-rule data needed by
   * antennaPoseJacobian().  Pass null when no Jacobian is required.
   */
  Point3 antennaPosition(const Pose3& pose,
                         PoseFrame* frame = nullptr) const;

  /**
   * Convert a 1x3 Jacobian (d/d antenna position) into a 1x6 Jacobian w.r.t.
   * the pose tangent space, using the cached frame from antennaPosition().
   */
  Matrix16 antennaPoseJacobian(const Matrix13& H_antenna,
                               const PoseFrame& frame) const;

  bool equals(const LeverArm& other, double tol) const;
};

}  // namespace gnss
}  // namespace gtsam
