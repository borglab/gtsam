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
 *   error = e . (v_s - v_r) + c * (ddt_r - ddt_s) - (-lambda * Doppler)
 *
 * where
 *   - e        is the unit line-of-sight vector from receiver to satellite,
 *   - v_s, v_r are the satellite and receiver ECEF velocities (m/s),
 *   - ddt_r    is the receiver clock drift (s/s) -- the estimated state,
 *   - ddt_s    is the satellite clock drift (s/s),
 *   - lambda * Doppler is the measured range rate (m/s); a positive Doppler
 *     (approaching satellite) corresponds to a decreasing range, hence the
 *     leading minus sign.
 *
 * The satellite velocity and the line-of-sight are held constant per factor;
 * their (second-order) dependence on the receiver position is neglected, which
 * is standard for Doppler velocity estimation.  Doppler provides receiver
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
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(DopplerFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(measRangeRate_);
    ar& BOOST_SERIALIZATION_NVP(satVel_);
    ar& BOOST_SERIALIZATION_NVP(los_);
    ar& BOOST_SERIALIZATION_NVP(satClkDrift_);
  }
#endif
};

/// traits
template <>
struct traits<DopplerFactor> : public Testable<DopplerFactor> {};

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
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(ClockDriftFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(dt_);
  }
#endif
};

/// traits
template <>
struct traits<ClockDriftFactor> : public Testable<ClockDriftFactor> {};

}  // namespace gtsam
