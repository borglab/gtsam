/**
 *  @file   PseudorangeFactor.h
 *  @author Sammy Guo
 *  @brief  Header file for GNSS Pseudorange factor
 *  @date   January 18, 2026
 **/
#pragma once

#include <gtsam/base/std_optional_serialization.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GnssCommon.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>
#include <string>

namespace gtsam {

/**
 * Base class storing common members for GNSS-related pseudorange factors.
 *
 * Aliased to the shared GnssMeasurementBase so that pseudorange and carrier
 * phase factors share a single representation.  In this alias the
 * `measurement_` field stores the pseudorange measurement in meters.
 */
using PseudorangeBase = GnssMeasurementBase;

/**
 * Simplified GNSS pseudorange model for basic positioning problems.
 *
 * This factor implements a simplified version of equation 5.6 [1]
 * \rho = r + c[\delta t_u - \delta t^s] + I_{\rho} + T_{\rho} + \epsilon_{\rho}
 * where `\rho` is measured pseudorange (in meters) from the receiver,
 * `r` true range (in meters) between receiver antenna and satellite,
 * `c` is speed of light in a vacuum (m/s),
 * `\delta t_u` is receiver clock bias (seconds),
 * `\delta t_s` is satellite clock bias (seconds),
 * and `I_{\rho}`, `T_{\rho}`, and `\epsilon_{\rho}` are ionospheric,
 * tropospheric, and unmodeled errors respectively.
 *
 * Ionospheric and tropospheric terms are omitted in this simplified factor.
 * Note that this factor is also designed for code-phase measurements.
 *
 * @ingroup navigation
 *
 * REFERENCES:
 * [1] P. Misra et. al., "Global Positioning Systems: Signals, Measurements, and
 * Performance", Second Edition, 2012.
 */
class GTSAM_EXPORT PseudorangeFactor : public NoiseModelFactorN<Point3, double>,
                                       private PseudorangeBase {
 private:
  typedef NoiseModelFactorN<Point3, double> Base;

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<PseudorangeFactor> shared_ptr;

  /// Typedef to this class
  typedef PseudorangeFactor This;

  /** default constructor - only use for serialization */
  PseudorangeFactor() = default;

  virtual ~PseudorangeFactor() = default;

  /**
   * Construct a PseudorangeFactor that models the distance between a receiver
   * and a satellite.
   *
   * @param receiverPositionKey Receiver gtsam::Point3 ECEF position node.
   * @param receiverClockBiasKey Receiver clock bias node.
   * @param measuredPseudorange Receiver-measured pseudorange in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param satelliteClockBias Satellite clock bias in seconds.
   * @param model 1-D pseudorange noise model.
   */
  PseudorangeFactor(
      Key receiverPositionKey, Key receiverClockBiasKey,
      double measuredPseudorange, const Point3& satellitePosition,
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
                       OptionalMatrixType HreceiverPos,
                       OptionalMatrixType HreceiverClockBias) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PseudorangeFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(measurement_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
  }
#endif
};

/// traits
template <>
struct traits<PseudorangeFactor> : public Testable<PseudorangeFactor> {};

/**
 * Simple differentially-corrected pseudorange factor for precise positioning.
 *
 * This factor implements the model prescribed by chapter 5.8.2 from [1],
 * where a reference GNSS receiver with known position provides differential
 * pseudorange corrections for a "user" receiver to eliminate common-mode
 * atmospheric errors. The idea being that spatially local receivers experience
 * the same atmospheric errors since their signal paths pass through the same
 * regions of Earth's atmosphere. Therefore, this factor accepts an additional
 * "differential correction" variable from a reference receiver to cancel-out
 * local-area biases from the user's pseudoranges.
 *
 * Note that this factor is designed for code-phase measurements.
 *
 * @example Please see the `DifferentialPseudorangeExample.ipynb` notebook
 * for a demonstration of this factor on CORS datasets.
 *
 * @ingroup navigation
 *
 * REFERENCES:
 * [1] P. Misra et. al., "Global Positioning Systems: Signals, Measurements, and
 * Performance", Second Edition, 2012.
 */
class GTSAM_EXPORT DifferentialPseudorangeFactor
    : public NoiseModelFactorN<Point3, double, double>,
      private PseudorangeBase {
 private:
  typedef NoiseModelFactorN<Point3, double, double> Base;

 public:
  using Base::evaluateError;
  typedef std::shared_ptr<DifferentialPseudorangeFactor> shared_ptr;
  typedef DifferentialPseudorangeFactor This;

  DifferentialPseudorangeFactor() = default;
  virtual ~DifferentialPseudorangeFactor() = default;

  DifferentialPseudorangeFactor(
      Key receiverPositionKey, Key receiverClockBiasKey,
      Key differentialCorrectionKey, double measuredPseudorange,
      const Point3& satellitePosition, double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  void print(const std::string& s = "", const KeyFormatter& keyFormatter =
                                            DefaultKeyFormatter) const override;
  bool equals(const NonlinearFactor& expected,
              double tol = 1e-9) const override;
  Vector evaluateError(
      const Point3& receiverPosition, const double& receiverClock_bias,
      const double& differentialCorrection, OptionalMatrixType HreceiverPos,
      OptionalMatrixType HreceiverClockBias,
      OptionalMatrixType HdifferentialCorrection) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DifferentialPseudorangeFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(measurement_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
  }
#endif
};

/// traits
template <>
struct traits<DifferentialPseudorangeFactor>
    : public Testable<DifferentialPseudorangeFactor> {};

/**
 * GNSS pseudorange factor with lever arm correction.
 *
 * Like PseudorangeFactor, but uses a Pose3 (position + attitude) as the
 * receiver state variable, allowing compensation for a lever arm offset
 * between the body frame origin and the GNSS antenna location.
 *
 * The antenna position is computed as:
 *   antenna_pos = ecef_T_body.translation() + ecef_R_body * leverArm
 *
 * where leverArm is the body-frame translation from the body origin to the
 * antenna. The error model is:
 *   error = ||antenna_pos - satPos|| + c*(dt_u - dt_s) - pseudorange
 *
 * When the optional ecef_T_nav transform is provided, the pose key is
 * interpreted as a local navigation frame pose (e.g., ENU), and the factor
 * internally converts it to ECEF via ecef_T_body = ecef_T_nav * nav_T_body.
 * This allows the same Pose3 variable to be shared with ImuFactor.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT PseudorangeFactorArm
    : public NoiseModelFactorN<Pose3, double>,
      private PseudorangeBase {
 private:
  typedef NoiseModelFactorN<Pose3, double> Base;

  gnss::LeverArm arm_;  ///< Lever arm + optional ecef_T_nav.

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<PseudorangeFactorArm> shared_ptr;

  /// Typedef to this class
  typedef PseudorangeFactorArm This;

  /** default constructor - only use for serialization */
  PseudorangeFactorArm() : PseudorangeBase{0.0, Point3(0, 0, 0), 0.0} {}

  virtual ~PseudorangeFactorArm() = default;

  /**
   * Construct a PseudorangeFactorArm (ECEF pose key).
   *
   * @param poseKey Receiver gtsam::Pose3 key (body pose in ECEF frame).
   * @param receiverClockBiasKey Receiver clock bias node.
   * @param measuredPseudorange Receiver-measured pseudorange in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param leverArm Translation from body origin to antenna in body frame.
   * @param satelliteClockBias Satellite clock bias in seconds.
   * @param model 1-D pseudorange noise model.
   */
  PseudorangeFactorArm(
      Key poseKey, Key receiverClockBiasKey,
      double measuredPseudorange, const Point3& satellitePosition,
      const Point3& leverArm, double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /**
   * Construct a PseudorangeFactorArm with ecef_T_nav (local nav frame pose key).
   *
   * @param poseKey Receiver gtsam::Pose3 key (body pose in local nav frame).
   * @param receiverClockBiasKey Receiver clock bias node.
   * @param measuredPseudorange Receiver-measured pseudorange in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param leverArm Translation from body origin to antenna in body frame.
   * @param ecef_T_nav Transform from local navigation frame to ECEF.
   * @param satelliteClockBias Satellite clock bias in seconds.
   * @param model 1-D pseudorange noise model.
   */
  PseudorangeFactorArm(
      Key poseKey, Key receiverClockBiasKey,
      double measuredPseudorange, const Point3& satellitePosition,
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
                       OptionalMatrixType H_pose,
                       OptionalMatrixType HreceiverClockBias) const override;

  /// return the lever arm, a position in the body frame
  inline const Point3& leverArm() const { return arm_.b; }

  /// return the optional ecef_T_nav transform
  inline const std::optional<Pose3>& ecefTnav() const { return arm_.ecef_T_nav; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PseudorangeFactorArm::Base);
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
struct traits<PseudorangeFactorArm>
    : public Testable<PseudorangeFactorArm> {};

/**
 * Differentially-corrected pseudorange factor with lever arm correction.
 *
 * Combines the differential correction model of DifferentialPseudorangeFactor
 * with the lever arm compensation of PseudorangeFactorArm. Uses a Pose3
 * (position + attitude) as the receiver state variable.
 *
 * The error model is:
 *   error = ||antenna_pos - satPos|| + c*(dt_u - dt_s) - pseudorange - correction
 *
 * When the optional ecef_T_nav transform is provided, the pose key is
 * interpreted as a local navigation frame pose (e.g., ENU), and the factor
 * internally converts it to ECEF via ecef_T_body = ecef_T_nav * nav_T_body.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DifferentialPseudorangeFactorArm
    : public NoiseModelFactorN<Pose3, double, double>,
      private PseudorangeBase {
 private:
  typedef NoiseModelFactorN<Pose3, double, double> Base;
  gnss::LeverArm arm_;

 public:
  using Base::evaluateError;
  typedef std::shared_ptr<DifferentialPseudorangeFactorArm> shared_ptr;
  typedef DifferentialPseudorangeFactorArm This;

  DifferentialPseudorangeFactorArm()
      : PseudorangeBase{0.0, Point3(0, 0, 0), 0.0} {}
  virtual ~DifferentialPseudorangeFactorArm() = default;

  DifferentialPseudorangeFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key differentialCorrectionKey,
      double measuredPseudorange, const Point3& satellitePosition,
      const Point3& leverArm, double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  DifferentialPseudorangeFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key differentialCorrectionKey,
      double measuredPseudorange, const Point3& satellitePosition,
      const Point3& leverArm, const Pose3& ecef_T_nav,
      double satelliteClockBias = 0.0,
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
                       const double& receiverClockBias,
                       const double& differentialCorrection,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType HreceiverClockBias,
                       OptionalMatrixType HdifferentialCorrection) const override;

  inline const Point3& leverArm() const { return arm_.b; }
  inline const std::optional<Pose3>& ecefTnav() const { return arm_.ecef_T_nav; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DifferentialPseudorangeFactorArm::Base);
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
struct traits<DifferentialPseudorangeFactorArm>
    : public Testable<DifferentialPseudorangeFactorArm> {};

/**
 * Double-difference pseudorange factor.
 *
 * Convenience factor that takes four raw (undifferenced) pseudorange
 * observations -- rover and base for both reference and target satellites --
 * along with satellite positions at rover and base observation times, and the
 * base station position.  The factor forms the double-difference internally
 * and computes geometric distances with Sagnac correction.
 *
 * Satellite positions differ between rover and base times because satellites
 * move ~3 km/s. Using the same positions for both causes ~100m DD errors
 * when rover and base observation times differ (e.g. rover 5Hz, base 1Hz).
 *
 * error = [(geodist(satRefRov,pos) - geodist(satRefBase,basePos))
 *        - (geodist(satTargetRov,pos) - geodist(satTargetBase,basePos))]
 *       - [(prRovRef - prBaseRef) - (prRovTarget - prBaseTarget)]
 *
 * Use this factor (instead of connecting four PseudorangeFactors through
 * shared receiver/satellite clock-bias variables) when:
 *   - the base station position is known (not being estimated) so that the
 *     two base-side ranges are constants;
 *   - receiver and satellite clock biases can be cancelled analytically via
 *     the DD; this removes three state variables from the graph.
 * For scenarios where receiver/satellite clock biases are also being
 * estimated, prefer composing several PseudorangeFactors instead.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DoubleDifferencePseudorangeFactor
    : public NoiseModelFactorN<Point3> {
 private:
  typedef NoiseModelFactorN<Point3> Base;

  gnss::DoubleDifferenceData dd_;  ///< Shared DD observation/geometry data.

 public:
  // Expose the convenience evaluateError overloads from NoiseModelFactorN
  // (e.g. the no-Jacobian and Matrix& variants used in tests).
  using Base::evaluateError;
  typedef std::shared_ptr<DoubleDifferencePseudorangeFactor> shared_ptr;
  typedef DoubleDifferencePseudorangeFactor This;

  DoubleDifferencePseudorangeFactor() = default;

  virtual ~DoubleDifferencePseudorangeFactor() = default;

  /**
   * @param positionKey   Rover Point3 ECEF position.
   * @param prRovRef      Rover pseudorange for ref satellite [m].
   * @param prBaseRef     Base pseudorange for ref satellite [m].
   * @param prRovTarget   Rover pseudorange for target satellite [m].
   * @param prBaseTarget  Base pseudorange for target satellite [m].
   * @param satRefRov     Ref satellite ECEF at rover observation time [m].
   * @param satTargetRov  Target satellite ECEF at rover observation time [m].
   * @param satRefBase    Ref satellite ECEF at base observation time [m].
   * @param satTargetBase Target satellite ECEF at base observation time [m].
   * @param basePos       Base station ECEF position [m].
   * @param model         1-D noise model.
   */
  DoubleDifferencePseudorangeFactor(
      Key positionKey,
      double prRovRef, double prBaseRef,
      double prRovTarget, double prBaseTarget,
      const Point3& satRefRov, const Point3& satTargetRov,
      const Point3& satRefBase, const Point3& satTargetBase,
      const Point3& basePos,
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
                       OptionalMatrixType H) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DoubleDifferencePseudorangeFactor::Base);
    ar& boost::serialization::make_nvp("prRovRef_", dd_.rovRef);
    ar& boost::serialization::make_nvp("prBaseRef_", dd_.baseRef);
    ar& boost::serialization::make_nvp("prRovTarget_", dd_.rovTarget);
    ar& boost::serialization::make_nvp("prBaseTarget_", dd_.baseTarget);
    ar& boost::serialization::make_nvp("satRefRov_", dd_.satRefRov);
    ar& boost::serialization::make_nvp("satTargetRov_", dd_.satTargetRov);
    ar& boost::serialization::make_nvp("satRefBase_", dd_.satRefBase);
    ar& boost::serialization::make_nvp("satTargetBase_", dd_.satTargetBase);
    ar& boost::serialization::make_nvp("basePos_", dd_.basePos);
  }
#endif
};

template <>
struct traits<DoubleDifferencePseudorangeFactor>
    : public Testable<DoubleDifferencePseudorangeFactor> {};

/**
 * Double-difference pseudorange factor with lever arm correction.
 *
 * Like DoubleDifferencePseudorangeFactor but uses Pose3 (position + attitude)
 * with lever arm offset. Optional ecef_T_nav for local navigation frame.
 *
 * @ingroup navigation
 */
class GTSAM_EXPORT DoubleDifferencePseudorangeFactorArm
    : public NoiseModelFactorN<Pose3> {
 private:
  typedef NoiseModelFactorN<Pose3> Base;

  gnss::DoubleDifferenceData dd_;
  gnss::LeverArm arm_;

 public:
  // Expose the convenience evaluateError overloads from NoiseModelFactorN
  // (e.g. the no-Jacobian and Matrix& variants used in tests).
  using Base::evaluateError;
  typedef std::shared_ptr<DoubleDifferencePseudorangeFactorArm> shared_ptr;
  typedef DoubleDifferencePseudorangeFactorArm This;

  DoubleDifferencePseudorangeFactorArm() = default;

  virtual ~DoubleDifferencePseudorangeFactorArm() = default;

  DoubleDifferencePseudorangeFactorArm(
      Key poseKey,
      double prRovRef, double prBaseRef,
      double prRovTarget, double prBaseTarget,
      const Point3& satRefRov, const Point3& satTargetRov,
      const Point3& satRefBase, const Point3& satTargetBase,
      const Point3& basePos, const Point3& leverArm,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  DoubleDifferencePseudorangeFactorArm(
      Key poseKey,
      double prRovRef, double prBaseRef,
      double prRovTarget, double prBaseTarget,
      const Point3& satRefRov, const Point3& satTargetRov,
      const Point3& satRefBase, const Point3& satTargetBase,
      const Point3& basePos, const Point3& leverArm,
      const Pose3& ecef_T_nav,
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
                       OptionalMatrixType H_pose) const override;

  inline const Point3& leverArm() const { return arm_.b; }
  inline const std::optional<Pose3>& ecefTnav() const { return arm_.ecef_T_nav; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DoubleDifferencePseudorangeFactorArm::Base);
    ar& boost::serialization::make_nvp("prRovRef_", dd_.rovRef);
    ar& boost::serialization::make_nvp("prBaseRef_", dd_.baseRef);
    ar& boost::serialization::make_nvp("prRovTarget_", dd_.rovTarget);
    ar& boost::serialization::make_nvp("prBaseTarget_", dd_.baseTarget);
    ar& boost::serialization::make_nvp("satRefRov_", dd_.satRefRov);
    ar& boost::serialization::make_nvp("satTargetRov_", dd_.satTargetRov);
    ar& boost::serialization::make_nvp("satRefBase_", dd_.satRefBase);
    ar& boost::serialization::make_nvp("satTargetBase_", dd_.satTargetBase);
    ar& boost::serialization::make_nvp("basePos_", dd_.basePos);
    ar& boost::serialization::make_nvp("bL_", arm_.b);
    ar& boost::serialization::make_nvp("ecef_T_nav_", arm_.ecef_T_nav);
  }
#endif
};

template <>
struct traits<DoubleDifferencePseudorangeFactorArm>
    : public Testable<DoubleDifferencePseudorangeFactorArm> {};

}  // namespace gtsam
