/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   DopplerFactor.h
 *  @brief  Header file for the GNSS Doppler (range-rate) factor
 *  @date   July 2026
 **/
#pragma once

#include <gtsam/base/std_optional_serialization.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/navigation/GnssCommon.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {

/**
 * GNSS Doppler (range-rate) factor.
 *
 * Constrains the receiver ECEF velocity and the receiver clock drift, where
 * the drift is not a separate state but the time difference of the clock-bias
 * states at adjacent epochs:
 *
 *   error = e . (v_s - v_r)
 *           + c * ((dt_r(k) - dt_r(k-1)) / dt - ddt_s)
 *           + sagnac_rate
 *           - (-lambda * Doppler)
 *
 * with e the unit line-of-sight vector (receiver -> satellite), v_s/v_r the
 * satellite/receiver ECEF velocities (m/s), dt_r the receiver clock biases
 * (s), dt = t_k - t_{k-1} (s), ddt_s the satellite clock drift (s/s), and
 * sagnac_rate the earth-rotation correction to the range rate.  The measured
 * range rate is -lambda * Doppler (positive Doppler = closing range).
 *
 * Satellite position/velocity and receiver position are constants per factor:
 * the line-of-sight is computed once from them and enters no Jacobian.
 *
 * The two clock-bias keys must be distinct states at adjacent epochs (the
 * same key twice forces zero drift), so Doppler factors start at the second
 * epoch. With per-constellation clock biases the drift is common to all
 * systems, so key every Doppler factor on a single clock-bias series.
 *
 * Keys: [velocity (Vector3, m/s),
 *        receiver clock bias at epoch k-1 (double, s),
 *        receiver clock bias at epoch k   (double, s)].
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DopplerFactor
    : public NoiseModelFactorT<Vector1, Vector3, double, double> {
 private:
  typedef NoiseModelFactorT<Vector1, Vector3, double, double> Base;

  double measRangeRate_ = 0.0;        ///< Measured range rate = -lambda*D [m/s].
  Point3 satVel_{0, 0, 0};            ///< Satellite ECEF velocity [m/s].
  Point3 los_{0, 0, 0};              ///< Unit LOS, receiver -> satellite.
  double satClkDrift_ = 0.0;          ///< Satellite clock drift [s/s].
  double dt_ = 1.0;                   ///< Epoch interval t_k - t_{k-1} [s].
  Point3 velSagnac_{0, 0, 0};         ///< Sagnac rate coeff, d(rate)/d(v_r).
  double sagnacOffset_ = 0.0;         ///< v_r-independent Sagnac rate term [m/s].

 public:
  using Base::evaluateError;
  typedef std::shared_ptr<DopplerFactor> shared_ptr;
  typedef DopplerFactor This;

  /** default constructor - only use for serialization */
  DopplerFactor() = default;
  virtual ~DopplerFactor() = default;

  /**
   * @param velocityKey         Receiver ECEF velocity node (Vector3, m/s).
   * @param clockBiasPrevKey    Receiver clock bias node at epoch k-1 (s).
   * @param clockBiasCurrKey    Receiver clock bias node at epoch k   (s).
   * @param measuredDoppler     Measured Doppler [Hz].
   * @param wavelength          Carrier wavelength [m/cycle].
   * @param satellitePosition   Satellite ECEF position [m] (for the LOS).
   * @param satelliteVelocity   Satellite ECEF velocity [m/s].
   * @param receiverPosition    Receiver ECEF position [m] (for the LOS).
   * @param dt                  Epoch interval t_k - t_{k-1} [s], must be > 0.
   * @param satelliteClockDrift Satellite clock drift [s/s].
   * @param model               1-D range-rate noise model.
   */
  DopplerFactor(Key velocityKey, Key clockBiasPrevKey, Key clockBiasCurrKey,
                double measuredDoppler, double wavelength,
                const Point3& satellitePosition,
                const Point3& satelliteVelocity, const Point3& receiverPosition,
                double dt, double satelliteClockDrift = 0.0,
                const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  /// equals
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  /// vector of errors
  Vector1 evaluateError(const Vector3& velocity, const double& clockBiasPrev,
                        const double& clockBiasCurr,
                        OptionalMatrixType Hvelocity,
                        OptionalMatrixType HclockBiasPrev,
                        OptionalMatrixType HclockBiasCurr) const override;

  /// Measured range rate (= -lambda * Doppler) [m/s].
  inline double measuredRangeRate() const { return measRangeRate_; }
  /// Unit line-of-sight vector (receiver -> satellite).
  inline const Point3& lineOfSight() const { return los_; }
  /// Epoch interval t_k - t_{k-1} [s].
  inline double dt() const { return dt_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(measRangeRate_);
    ar& BOOST_SERIALIZATION_NVP(satVel_);
    ar& BOOST_SERIALIZATION_NVP(los_);
    ar& BOOST_SERIALIZATION_NVP(satClkDrift_);
    ar& BOOST_SERIALIZATION_NVP(dt_);
    ar& BOOST_SERIALIZATION_NVP(velSagnac_);
    ar& BOOST_SERIALIZATION_NVP(sagnacOffset_);
  }
#endif
};

/// traits
template <>
struct traits<DopplerFactor> : public Testable<DopplerFactor> {};

/**
 * DopplerFactor with a kinematic lever-arm correction, keyed on a body Pose3.
 *
 * When the body rotates at angular rate omega, the antenna moves relative to
 * the body origin, so the range-rate error uses
 *
 *   v_antenna = v_body + ecef_R_body * (omega x leverArm)
 *
 * in place of v_r, with omega the measured body-frame angular velocity and
 * leverArm the body-frame antenna offset.  Everything else (clock-drift term,
 * line-of-sight and Sagnac terms at the nominal receiver position) matches
 * DopplerFactor; the pose enters only through its attitude, and with
 * omega = 0 the factor reduces to DopplerFactor.
 *
 * With the optional ecef_T_nav transform the pose key is a local nav-frame
 * pose (e.g. ENU); `velocity` is then also nav-frame and rotated to ECEF by
 * ecef_R_nav, like the other lever-arm factors.  Without it, both are ECEF.
 *
 * Keys: [pose (Pose3), velocity (Vector3, ECEF m/s; nav-frame with ecef_T_nav),
 *        clock bias k-1 (double, s), clock bias k (double, s)].
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DopplerFactorArm
    : public NoiseModelFactorN<Pose3, Vector3, double, double> {
 private:
  typedef NoiseModelFactorN<Pose3, Vector3, double, double> Base;

  double measRangeRate_ = 0.0;  ///< Measured range rate = -lambda*D [m/s].
  Point3 satVel_{0, 0, 0};      ///< Satellite ECEF velocity [m/s].
  Point3 los_{0, 0, 0};        ///< Unit LOS, receiver -> satellite.
  double satClkDrift_ = 0.0;    ///< Satellite clock drift [s/s].
  double dt_ = 1.0;             ///< Epoch interval t_k - t_{k-1} [s].
  Point3 velSagnac_{0, 0, 0};   ///< Sagnac rate coeff, d(rate)/d(v_ant).
  double sagnacOffset_ = 0.0;   ///< v_ant-independent Sagnac rate term [m/s].
  gnss::LeverArm arm_;          ///< Lever arm (body frame) + optional ecef_T_nav.
  Point3 leverVel_{0, 0, 0};    ///< omega x leverArm (body frame) [m/s].

 public:
  using Base::evaluateError;
  typedef std::shared_ptr<DopplerFactorArm> shared_ptr;
  typedef DopplerFactorArm This;

  /** default constructor - only use for serialization */
  DopplerFactorArm() = default;
  virtual ~DopplerFactorArm() = default;

  /**
   * Construct a DopplerFactorArm with an ECEF pose key.
   *
   * @param poseKey             Receiver body Pose3 (ECEF) node.
   * @param velocityKey         Receiver ECEF velocity node (Vector3, m/s).
   * @param clockBiasPrevKey    Receiver clock bias node at epoch k-1 (s).
   * @param clockBiasCurrKey    Receiver clock bias node at epoch k   (s).
   * @param measuredDoppler     Measured Doppler [Hz].
   * @param wavelength          Carrier wavelength [m/cycle].
   * @param satellitePosition   Satellite ECEF position [m] (for the LOS).
   * @param satelliteVelocity   Satellite ECEF velocity [m/s].
   * @param receiverPosition    Nominal receiver ECEF position [m] (for the LOS).
   * @param leverArm            Antenna lever arm in the body frame [m].
   * @param angularVelocity     Body-frame angular velocity omega [rad/s].
   * @param dt                  Epoch interval t_k - t_{k-1} [s], must be > 0.
   * @param satelliteClockDrift Satellite clock drift [s/s].
   * @param model               1-D range-rate noise model.
   */
  DopplerFactorArm(Key poseKey, Key velocityKey, Key clockBiasPrevKey,
                   Key clockBiasCurrKey, double measuredDoppler,
                   double wavelength, const Point3& satellitePosition,
                   const Point3& satelliteVelocity,
                   const Point3& receiverPosition, const Point3& leverArm,
                   const Point3& angularVelocity, double dt,
                   double satelliteClockDrift = 0.0,
                   const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /// Construct with a local nav-frame pose key + ecef_T_nav; `velocity` is then
  /// a nav-frame velocity (rotated to ECEF by ecef_R_nav).
  DopplerFactorArm(Key poseKey, Key velocityKey, Key clockBiasPrevKey,
                   Key clockBiasCurrKey, double measuredDoppler,
                   double wavelength, const Point3& satellitePosition,
                   const Point3& satelliteVelocity,
                   const Point3& receiverPosition, const Point3& leverArm,
                   const Pose3& ecef_T_nav, const Point3& angularVelocity,
                   double dt, double satelliteClockDrift = 0.0,
                   const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  /// equals
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  /// vector of errors
  Vector evaluateError(const Pose3& pose, const Vector3& velocity,
                       const double& clockBiasPrev, const double& clockBiasCurr,
                       OptionalMatrixType Hpose, OptionalMatrixType Hvelocity,
                       OptionalMatrixType HclockBiasPrev,
                       OptionalMatrixType HclockBiasCurr) const override;

  /// Measured range rate (= -lambda * Doppler) [m/s].
  inline double measuredRangeRate() const { return measRangeRate_; }
  /// Unit line-of-sight vector (receiver -> satellite).
  inline const Point3& lineOfSight() const { return los_; }
  /// Epoch interval t_k - t_{k-1} [s].
  inline double dt() const { return dt_; }
  /// Lever arm in the body frame [m].
  inline const Point3& leverArm() const { return arm_.b; }
  /// Optional ECEF-from-nav transform.
  inline const std::optional<Pose3>& ecefTnav() const {
    return arm_.ecef_T_nav;
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(measRangeRate_);
    ar& BOOST_SERIALIZATION_NVP(satVel_);
    ar& BOOST_SERIALIZATION_NVP(los_);
    ar& BOOST_SERIALIZATION_NVP(satClkDrift_);
    ar& BOOST_SERIALIZATION_NVP(dt_);
    ar& BOOST_SERIALIZATION_NVP(velSagnac_);
    ar& BOOST_SERIALIZATION_NVP(sagnacOffset_);
    ar& boost::serialization::make_nvp("bL_", arm_.b);
    ar& boost::serialization::make_nvp("ecef_T_nav_", arm_.ecef_T_nav);
    ar& BOOST_SERIALIZATION_NVP(leverVel_);
  }
#endif
};

/// traits
template <>
struct traits<DopplerFactorArm> : public Testable<DopplerFactorArm> {};

}  // namespace gtsam
