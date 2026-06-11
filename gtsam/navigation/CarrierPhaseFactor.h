/**
 *  @file   CarrierPhaseFactor.h
 *  @brief  Header file for GNSS Carrier Phase factors
 *  @date   March 23, 2026
 **/
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GnssCommon.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>
#include <string>

namespace gtsam {

/**
 * Base class storing common members for carrier phase factors.
 *
 * Aliased to the shared GnssMeasurementBase.  In this alias the `measurement_`
 * field stores the carrier phase measurement in meters (= lambda * phi_cycles).
 *
 * Clock biases are in seconds (same convention as PseudorangeFactorArm).
 * Ambiguity is in meters (= lambda * N). To recover the integer
 * ambiguity N, divide by the wavelength: N = ambiguity_meters / lambda
 */
using CarrierPhaseBase = GnssMeasurementBase;

/**
 * Undifferenced GNSS carrier phase factor for point positioning.
 *
 * The error model is:
 *   error = ||recv_pos - satPos|| + c*(dt_u - dt_s) + ambiguity - phi
 *
 * where dt_u, dt_s are in seconds and ambiguity is in meters.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseFactor
    : public NoiseModelFactorN<Point3, double, double>,
      private CarrierPhaseBase {
 private:
  typedef NoiseModelFactorN<Point3, double, double> Base;

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseFactor> shared_ptr;
  typedef CarrierPhaseFactor This;

  /** default constructor - only use for serialization */
  CarrierPhaseFactor()
      : CarrierPhaseBase{0.0, Point3(0, 0, 0), 0.0} {}

  virtual ~CarrierPhaseFactor() = default;

  /**
   * Construct a CarrierPhaseFactor.
   *
   * @param receiverPositionKey Receiver gtsam::Point3 ECEF position node.
   * @param receiverClockBiasKey Receiver clock bias node (seconds).
   * @param ambiguityKey Ambiguity node (meters, = lambda * N).
   * @param measuredCarrierPhaseMeters Carrier phase measurement in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param satelliteClockBias Satellite clock bias in seconds.
   * @param model 1-D noise model.
   */
  CarrierPhaseFactor(
      Key receiverPositionKey, Key receiverClockBiasKey, Key ambiguityKey,
      double measuredCarrierPhaseMeters, const Point3& satellitePosition,
      double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /// @return a deep copy of this factor
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
  Vector evaluateError(const Point3& receiverPosition,
                       const double& receiverClockBias,
                       const double& ambiguity,
                       OptionalMatrixType HreceiverPos,
                       OptionalMatrixType HreceiverClockBias,
                       OptionalMatrixType Hambiguity) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(measurement_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseFactor> : public Testable<CarrierPhaseFactor> {};

/**
 * Carrier phase factor with lever arm correction.
 *
 * Like CarrierPhaseFactor, but uses a Pose3 (position + attitude) as the
 * receiver state variable, allowing compensation for a lever arm offset.
 *
 * The antenna position is computed as:
 *   antenna_pos = ecef_T_body.translation() + ecef_R_body * leverArm
 *
 * The error model is:
 *   error = ||antenna_pos - satPos|| + c*(dt_u - dt_s) + ambiguity - phi
 *
 * When the optional ecef_T_nav transform is provided, the pose key is
 * interpreted as a local navigation frame pose (e.g., ENU), and the factor
 * internally converts it to ECEF via ecef_T_body = ecef_T_nav * nav_T_body.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT CarrierPhaseFactorArm
    : public NoiseModelFactorN<Pose3, double, double>,
      private CarrierPhaseBase {
 private:
  typedef NoiseModelFactorN<Pose3, double, double> Base;

  gnss::LeverArm arm_;

 public:
  using Base::evaluateError;

  typedef std::shared_ptr<CarrierPhaseFactorArm> shared_ptr;
  typedef CarrierPhaseFactorArm This;

  /** default constructor - only use for serialization */
  CarrierPhaseFactorArm() : CarrierPhaseBase{0.0, Point3(0, 0, 0), 0.0} {}

  virtual ~CarrierPhaseFactorArm() = default;

  /**
   * Construct a CarrierPhaseFactorArm (ECEF pose key).
   */
  CarrierPhaseFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key ambiguityKey,
      double measuredCarrierPhaseMeters, const Point3& satellitePosition,
      const Point3& leverArm, double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /**
   * Construct a CarrierPhaseFactorArm with ecef_T_nav (local nav frame pose).
   */
  CarrierPhaseFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key ambiguityKey,
      double measuredCarrierPhaseMeters, const Point3& satellitePosition,
      const Point3& leverArm, const Pose3& ecef_T_nav,
      double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /// @return a deep copy of this factor
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
  Vector evaluateError(const Pose3& pose,
                       const double& receiverClockBias,
                       const double& ambiguity,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType HreceiverClockBias,
                       OptionalMatrixType Hambiguity) const override;

  /// return the lever arm
  inline const Point3& leverArm() const { return arm_.b; }

  /// return the optional ecef_T_nav transform
  inline const std::optional<Pose3>& ecefTnav() const { return arm_.ecef_T_nav; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(CarrierPhaseFactorArm::Base);
    ar& BOOST_SERIALIZATION_NVP(measurement_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
    ar& boost::serialization::make_nvp("bL_", arm_.b);
    ar& boost::serialization::make_nvp("ecef_T_nav_", arm_.ecef_T_nav);
  }
#endif
};

/// traits
template <>
struct traits<CarrierPhaseFactorArm>
    : public Testable<CarrierPhaseFactorArm> {};

/**
 * Double-difference carrier phase factor.
 *
 * This factor is a convenience wrapper that takes four raw (undifferenced)
 * carrier phase observations -- rover and base for both reference and target
 * satellites -- along with satellite positions at both rover and base
 * observation times, the base station position, and the wavelength.  The
 * factor forms the double-difference internally.
 *
 * error = [(geodist(satRefRov,pos) - geodist(satRefBase,basePos))
 *        - (geodist(satTargetRov,pos) - geodist(satTargetBase,basePos))]
 *       + lam * (ambRef - ambTarget)
 *       - [(cpRovRefMeters - cpBaseRefMeters)
 *        - (cpRovTargetMeters - cpBaseTargetMeters)]
 *
 * Use this factor (instead of connecting four CarrierPhaseFactors through
 * shared receiver/satellite clock-bias variables) when:
 *   - the base station position is known (not being estimated) so that the
 *     two base-side ranges are constants;
 *   - receiver and satellite clock biases can be cancelled analytically via
 *     the DD; this removes three state variables from the graph.
 * The result is a small graph (only rover position + two ambiguities) that
 * is well-suited for fixed-baseline RTK post-processing and for use with an
 * external integer ambiguity resolver (e.g. LAMBDA) that expects DD
 * residuals.
 *
 * For scenarios where receiver/satellite clock biases are also being
 * estimated, prefer composing several CarrierPhaseFactors instead.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DoubleDifferenceCarrierPhaseFactor
    : public NoiseModelFactorN<Point3, double, double> {
 private:
  typedef NoiseModelFactorN<Point3, double, double> Base;

  gnss::DoubleDifferenceData dd_;
  double lam_ = 0;

 public:
  // Expose the convenience evaluateError overloads from NoiseModelFactorN
  // (e.g. the no-Jacobian and Matrix& variants used in tests).
  using Base::evaluateError;
  typedef std::shared_ptr<DoubleDifferenceCarrierPhaseFactor> shared_ptr;
  typedef DoubleDifferenceCarrierPhaseFactor This;

  DoubleDifferenceCarrierPhaseFactor() = default;

  virtual ~DoubleDifferenceCarrierPhaseFactor() = default;

  DoubleDifferenceCarrierPhaseFactor(
      Key positionKey, Key ambRefKey, Key ambTargetKey,
      double cpRovRefMeters, double cpBaseRefMeters,
      double cpRovTargetMeters, double cpBaseTargetMeters,
      const Point3& satRefRov, const Point3& satTargetRov,
      const Point3& satRefBase, const Point3& satTargetBase,
      const Point3& basePos, double lam,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Point3& pos,
                       const double& ambRef, const double& ambTarget,
                       OptionalMatrixType Hpos,
                       OptionalMatrixType HambRef,
                       OptionalMatrixType HambTarget) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DoubleDifferenceCarrierPhaseFactor::Base);
    ar& boost::serialization::make_nvp("cpRovRef_", dd_.rovRef);
    ar& boost::serialization::make_nvp("cpBaseRef_", dd_.baseRef);
    ar& boost::serialization::make_nvp("cpRovTarget_", dd_.rovTarget);
    ar& boost::serialization::make_nvp("cpBaseTarget_", dd_.baseTarget);
    ar& boost::serialization::make_nvp("satRefRov_", dd_.satRefRov);
    ar& boost::serialization::make_nvp("satTargetRov_", dd_.satTargetRov);
    ar& boost::serialization::make_nvp("satRefBase_", dd_.satRefBase);
    ar& boost::serialization::make_nvp("satTargetBase_", dd_.satTargetBase);
    ar& boost::serialization::make_nvp("basePos_", dd_.basePos);
    ar& BOOST_SERIALIZATION_NVP(lam_);
  }
#endif
};

template <>
struct traits<DoubleDifferenceCarrierPhaseFactor>
    : public Testable<DoubleDifferenceCarrierPhaseFactor> {};

/**
 * Double-difference carrier phase factor with lever arm correction.
 *
 * Like DoubleDifferenceCarrierPhaseFactor but uses Pose3 (position + attitude)
 * with lever arm offset. Optional ecef_T_nav for local navigation frame.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DoubleDifferenceCarrierPhaseFactorArm
    : public NoiseModelFactorN<Pose3, double, double> {
 private:
  typedef NoiseModelFactorN<Pose3, double, double> Base;

  gnss::DoubleDifferenceData dd_;
  double lam_ = 0;
  gnss::LeverArm arm_;

 public:
  // Expose the convenience evaluateError overloads from NoiseModelFactorN
  // (e.g. the no-Jacobian and Matrix& variants used in tests).
  using Base::evaluateError;
  typedef std::shared_ptr<DoubleDifferenceCarrierPhaseFactorArm> shared_ptr;
  typedef DoubleDifferenceCarrierPhaseFactorArm This;

  DoubleDifferenceCarrierPhaseFactorArm() = default;

  virtual ~DoubleDifferenceCarrierPhaseFactorArm() = default;

  DoubleDifferenceCarrierPhaseFactorArm(
      Key poseKey, Key ambRefKey, Key ambTargetKey,
      double cpRovRefMeters, double cpBaseRefMeters,
      double cpRovTargetMeters, double cpBaseTargetMeters,
      const Point3& satRefRov, const Point3& satTargetRov,
      const Point3& satRefBase, const Point3& satTargetBase,
      const Point3& basePos, double lam,
      const Point3& leverArm,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  DoubleDifferenceCarrierPhaseFactorArm(
      Key poseKey, Key ambRefKey, Key ambTargetKey,
      double cpRovRefMeters, double cpBaseRefMeters,
      double cpRovTargetMeters, double cpBaseTargetMeters,
      const Point3& satRefRov, const Point3& satTargetRov,
      const Point3& satRefBase, const Point3& satTargetBase,
      const Point3& basePos, double lam,
      const Point3& leverArm, const Pose3& ecef_T_nav,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;

  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;

  Vector evaluateError(const Pose3& pose,
                       const double& ambRef, const double& ambTarget,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType HambRef,
                       OptionalMatrixType HambTarget) const override;

  inline const Point3& leverArm() const { return arm_.b; }
  inline const std::optional<Pose3>& ecefTnav() const { return arm_.ecef_T_nav; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DoubleDifferenceCarrierPhaseFactorArm::Base);
    ar& boost::serialization::make_nvp("cpRovRef_", dd_.rovRef);
    ar& boost::serialization::make_nvp("cpBaseRef_", dd_.baseRef);
    ar& boost::serialization::make_nvp("cpRovTarget_", dd_.rovTarget);
    ar& boost::serialization::make_nvp("cpBaseTarget_", dd_.baseTarget);
    ar& boost::serialization::make_nvp("satRefRov_", dd_.satRefRov);
    ar& boost::serialization::make_nvp("satTargetRov_", dd_.satTargetRov);
    ar& boost::serialization::make_nvp("satRefBase_", dd_.satRefBase);
    ar& boost::serialization::make_nvp("satTargetBase_", dd_.satTargetBase);
    ar& boost::serialization::make_nvp("basePos_", dd_.basePos);
    ar& BOOST_SERIALIZATION_NVP(lam_);
    ar& boost::serialization::make_nvp("bL_", arm_.b);
    ar& boost::serialization::make_nvp("ecef_T_nav_", arm_.ecef_T_nav);
  }
#endif
};

template <>
struct traits<DoubleDifferenceCarrierPhaseFactorArm>
    : public Testable<DoubleDifferenceCarrierPhaseFactorArm> {};

}  // namespace gtsam
