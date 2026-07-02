/**
 *  @file   DopplerFactor.h
 *  @brief  Header file for the GNSS Doppler (range-rate) factor
 *  @date   June 17, 2026
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
 * Relates a measured Doppler observation to the receiver velocity and clock
 * drift, reusing the line-of-sight unit vector from gnss::geodist:
 *
 *   error = e . (v_s - v_r) + c * (ddt_r - ddt_s) + sagnac_rate
 *           - (-lambda * Doppler)
 *
 * where
 *   - e        is the unit line-of-sight vector from receiver to satellite,
 *   - v_s, v_r are the satellite and receiver ECEF velocities (m/s),
 *   - ddt_r    is the receiver clock drift (s/s) -- the estimated state,
 *   - ddt_s    is the satellite clock drift (s/s),
 *   - sagnac_rate = (OMGE/c) * (v_s.y*r_r.x + r_s.y*v_r.x
 *                               - v_s.x*r_r.y - r_s.x*v_r.y)
 *     is the earth-rotation (Sagnac) rate correction -- the time derivative of
 *     the Sagnac range term, matching RTKLIB's resdop(),
 *   - lambda * Doppler is the measured range rate (m/s); a positive Doppler
 *     (approaching satellite) corresponds to a decreasing range, hence the
 *     leading minus sign.
 *
 * The satellite position/velocity and the line-of-sight are held constant per
 * factor; only the (second-order) dependence of the line-of-sight on the
 * receiver position is neglected, which is standard for Doppler velocity
 * estimation.  Doppler provides receiver
 * velocity and clock-drift observability that is robust through carrier-phase
 * cycle slips and re-convergence.
 *
 * Keys: [velocity (Vector3, m/s), receiver clock drift (double, s/s)].
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DopplerFactor : public NoiseModelFactorN<Vector3, double> {
 private:
  typedef NoiseModelFactorN<Vector3, double> Base;

  double measRangeRate_ = 0.0;        ///< Measured range rate = -lambda*D [m/s].
  Point3 satVel_{0, 0, 0};            ///< Satellite ECEF velocity [m/s].
  Point3 los_{0, 0, 0};              ///< Unit LOS, receiver -> satellite.
  double satClkDrift_ = 0.0;          ///< Satellite clock drift [s/s].
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
   * @param clockDriftKey       Receiver clock drift node (double, s/s).
   * @param measuredDoppler     Measured Doppler [Hz].
   * @param wavelength          Carrier wavelength [m/cycle].
   * @param satellitePosition   Satellite ECEF position [m] (for the LOS).
   * @param satelliteVelocity   Satellite ECEF velocity [m/s].
   * @param receiverPosition    Receiver ECEF position [m] (for the LOS).
   * @param satelliteClockDrift Satellite clock drift [s/s].
   * @param model               1-D range-rate noise model.
   */
  DopplerFactor(Key velocityKey, Key clockDriftKey, double measuredDoppler,
                double wavelength, const Point3& satellitePosition,
                const Point3& satelliteVelocity, const Point3& receiverPosition,
                double satelliteClockDrift = 0.0,
                const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Vector3& velocity, const double& clockDrift,
                       OptionalMatrixType Hvelocity,
                       OptionalMatrixType HclockDrift) const override;

  /// Measured range rate (= -lambda * Doppler) [m/s].
  inline double measuredRangeRate() const { return measRangeRate_; }
  /// Unit line-of-sight vector (receiver -> satellite).
  inline const Point3& lineOfSight() const { return los_; }

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
    ar& BOOST_SERIALIZATION_NVP(velSagnac_);
    ar& BOOST_SERIALIZATION_NVP(sagnacOffset_);
  }
#endif
};

/// traits
template <>
struct traits<DopplerFactor> : public Testable<DopplerFactor> {};

/**
 * GNSS Doppler factor with lever-arm correction.
 *
 * Like DopplerFactor, but keys on a body Pose3 so the antenna's lever arm can
 * be accounted for.  Unlike the pseudorange/carrier lever-arm factors (which
 * correct the antenna *position*), the dominant lever-arm effect on a Doppler
 * (range-rate) observation is *kinematic*: when the body rotates at angular
 * rate omega, the antenna moves relative to the body origin, so
 *
 *   v_antenna = v_body + ecef_R_body * (omega x leverArm)
 *
 * where omega is the (measured) body-frame angular velocity and leverArm is the
 * body-frame antenna offset.  The range-rate error then uses v_antenna:
 *
 *   error = e . (v_s - v_antenna) + c*(ddt_r - ddt_s) + sagnac_rate
 *         - (-lambda*Doppler)
 *
 * The line-of-sight e and the Sagnac terms are evaluated at the provided
 * (nominal) receiver position, exactly as in DopplerFactor -- the second-order
 * dependence of the LOS on the pose translation is neglected (standard for
 * Doppler velocity estimation), so the pose enters only through its attitude.
 * With omega = 0 the factor reduces to DopplerFactor evaluated at `velocity`.
 *
 * When the optional ecef_T_nav transform is provided, the pose key is a local
 * navigation-frame pose (e.g. ENU) and ecef_R_body = ecef_R_nav * nav_R_body;
 * `velocity` is still the receiver ECEF velocity.
 *
 * Keys: [pose (Pose3), velocity (Vector3, ECEF m/s), clock drift (double, s/s)].
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DopplerFactorArm
    : public NoiseModelFactorN<Pose3, Vector3, double> {
 private:
  typedef NoiseModelFactorN<Pose3, Vector3, double> Base;

  double measRangeRate_ = 0.0;  ///< Measured range rate = -lambda*D [m/s].
  Point3 satVel_{0, 0, 0};      ///< Satellite ECEF velocity [m/s].
  Point3 los_{0, 0, 0};        ///< Unit LOS, receiver -> satellite.
  double satClkDrift_ = 0.0;    ///< Satellite clock drift [s/s].
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
   * @param clockDriftKey       Receiver clock drift node (double, s/s).
   * @param measuredDoppler     Measured Doppler [Hz].
   * @param wavelength          Carrier wavelength [m/cycle].
   * @param satellitePosition   Satellite ECEF position [m] (for the LOS).
   * @param satelliteVelocity   Satellite ECEF velocity [m/s].
   * @param receiverPosition    Nominal receiver ECEF position [m] (for the LOS).
   * @param leverArm            Antenna lever arm in the body frame [m].
   * @param angularVelocity     Body-frame angular velocity omega [rad/s].
   * @param satelliteClockDrift Satellite clock drift [s/s].
   * @param model               1-D range-rate noise model.
   */
  DopplerFactorArm(Key poseKey, Key velocityKey, Key clockDriftKey,
                   double measuredDoppler, double wavelength,
                   const Point3& satellitePosition,
                   const Point3& satelliteVelocity,
                   const Point3& receiverPosition, const Point3& leverArm,
                   const Point3& angularVelocity,
                   double satelliteClockDrift = 0.0,
                   const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /// Construct with a local nav-frame pose key + ecef_T_nav.
  DopplerFactorArm(Key poseKey, Key velocityKey, Key clockDriftKey,
                   double measuredDoppler, double wavelength,
                   const Point3& satellitePosition,
                   const Point3& satelliteVelocity,
                   const Point3& receiverPosition, const Point3& leverArm,
                   const Pose3& ecef_T_nav, const Point3& angularVelocity,
                   double satelliteClockDrift = 0.0,
                   const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Pose3& pose, const Vector3& velocity,
                       const double& clockDrift, OptionalMatrixType Hpose,
                       OptionalMatrixType Hvelocity,
                       OptionalMatrixType HclockDrift) const override;

  /// Measured range rate (= -lambda * Doppler) [m/s].
  inline double measuredRangeRate() const { return measRangeRate_; }
  /// Unit line-of-sight vector (receiver -> satellite).
  inline const Point3& lineOfSight() const { return los_; }
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

/**
 * Constant-drift receiver clock factor.
 *
 * Links two consecutive receiver clock biases through a (constant) clock drift,
 * i.e. the constant-velocity clock model -- the scalar analog of how the IMU
 * factor links position and velocity:
 *
 *   error = bias(k) - bias(k-1) - drift * dt
 *
 * The drift state is shared by the DopplerFactor (which observes it) and by this
 * factor (which integrates it into the clock bias used by the pseudorange /
 * carrier-phase factors), so Doppler velocity information propagates into the
 * clock bias. Bias is in seconds, drift in seconds/second, dt in seconds.
 *
 * Keys: [clock bias prev (double), clock bias curr (double), drift (double)].
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT ClockDriftFactor
    : public NoiseModelFactorN<double, double, double> {
 private:
  typedef NoiseModelFactorN<double, double, double> Base;
  double dt_ = 0.0;  ///< Time step between the two clock-bias epochs [s].

 public:
  using Base::evaluateError;
  typedef std::shared_ptr<ClockDriftFactor> shared_ptr;
  typedef ClockDriftFactor This;

  ClockDriftFactor() = default;
  virtual ~ClockDriftFactor() = default;

  /**
   * @param clockBiasPrevKey Receiver clock bias at epoch k-1 [s].
   * @param clockBiasCurrKey Receiver clock bias at epoch k   [s].
   * @param clockDriftKey    Receiver clock drift [s/s].
   * @param dt               Time step (t_k - t_{k-1}) [s].
   * @param model            1-D noise model (clock process noise).
   */
  ClockDriftFactor(Key clockBiasPrevKey, Key clockBiasCurrKey,
                   Key clockDriftKey, double dt,
                   const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const double& clockBiasPrev, const double& clockBiasCurr,
                       const double& clockDrift, OptionalMatrixType HbiasPrev,
                       OptionalMatrixType HbiasCurr,
                       OptionalMatrixType Hdrift) const override;

  inline double dt() const { return dt_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(dt_);
  }
#endif
};

/// traits
template <>
struct traits<ClockDriftFactor> : public Testable<ClockDriftFactor> {};

}  // namespace gtsam
