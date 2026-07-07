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
 * Relates a measured Doppler observation to the receiver velocity and the
 * receiver clock, reusing the line-of-sight unit vector from gnss::geodist.
 * The receiver clock drift is not a separate state: it is expressed as the
 * time difference of the two adjacent receiver clock-bias states already
 * estimated by the pseudorange / carrier-phase factors,
 *
 *   drift ~= (dt_r(k) - dt_r(k-1)) / dt
 *
 * so the factor reads
 *
 *   error = e . (v_s - v_r)
 *           + c * ((dt_r(k) - dt_r(k-1)) / dt - ddt_s)
 *           + sagnac_rate
 *           - (-lambda * Doppler)
 *
 * where
 *   - e        is the unit line-of-sight vector from receiver to satellite,
 *   - v_s, v_r are the satellite and receiver ECEF velocities (m/s),
 *   - dt_r(k-1), dt_r(k) are the receiver clock biases (s) at the previous
 *     and current epochs -- the estimated states,
 *   - dt       is the epoch interval t_k - t_{k-1} (s),
 *   - ddt_s    is the satellite clock drift (s/s),
 *   - sagnac_rate = (OMGE/c) * (v_s.y*r_r.x + r_s.y*v_r.x
 *                               - v_s.x*r_r.y - r_s.x*v_r.y)
 *     is the earth-rotation (Sagnac) correction to the range rate,
 *   - lambda * Doppler is the measured range rate (m/s); a positive Doppler
 *     (approaching satellite) corresponds to a decreasing range, hence the
 *     leading minus sign.
 *
 * Keying on the two clock biases (instead of a dedicated clock-drift state)
 * keeps the state vector identical to the pseudorange-only problem and needs
 * no extra between-epoch clock factor: the Doppler measurement itself
 * constrains the clock-bias evolution.  The instantaneous drift is
 * approximated by its average over [t_{k-1}, t_k].
 *
 * The satellite position/velocity and the receiver position are inputs held
 * constant per factor; the line-of-sight is computed once from them, so it does
 * not depend on any state and enters no Jacobian.
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
    : public NoiseModelFactorN<Vector3, double, double> {
 private:
  typedef NoiseModelFactorN<Vector3, double, double> Base;

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

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Vector3& velocity, const double& clockBiasPrev,
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
 *   error = e . (v_s - v_antenna)
 *         + c * ((dt_r(k) - dt_r(k-1)) / dt - ddt_s)
 *         + sagnac_rate - (-lambda*Doppler)
 *
 * with the receiver clock drift expressed through the two adjacent clock-bias
 * states, exactly as in DopplerFactor.
 *
 * The line-of-sight e and the Sagnac terms are evaluated at the provided
 * (nominal) receiver position, exactly as in DopplerFactor -- the second-order
 * dependence of the LOS on the pose translation is neglected, so the pose
 * enters only through its attitude.  With omega = 0 the factor reduces to
 * DopplerFactor evaluated at `velocity`.
 *
 * When the optional ecef_T_nav transform is provided, the pose key is a local
 * navigation-frame pose (e.g. ENU) and ecef_R_body = ecef_R_nav * nav_R_body;
 * `velocity` is still the receiver ECEF velocity.
 *
 * Keys: [pose (Pose3), velocity (Vector3, ECEF m/s),
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

  /// Construct with a local nav-frame pose key + ecef_T_nav.
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

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

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
