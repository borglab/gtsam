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
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <optional>
#include <string>

namespace gtsam {

/**
 * Base class storing common members for GNSS-related pseudorange factors.
 */
struct PseudorangeBase {
  double
      pseudorange_;    ///< Receiver-reported pseudorange measurement in meters.
  Point3 satPos_;      ///< Satellite position in WGS84 ECEF meters.
  double satClkBias_;  ///< Satellite clock bias in seconds.
};

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
    ar& BOOST_SERIALIZATION_NVP(pseudorange_);
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
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<DifferentialPseudorangeFactor> shared_ptr;

  /// Typedef to this class
  typedef DifferentialPseudorangeFactor This;

  /** default constructor - only use for serialization */
  DifferentialPseudorangeFactor() = default;

  virtual ~DifferentialPseudorangeFactor() = default;

  /**
   * Construct a DifferentialPseudorangeFactor that includes a
   * differential-correction term in its model for distance between a receiver
   * and a satellite.
   *
   * @param receiverPositionKey Receiver gtsam::Point3 ECEF position node.
   * @param receiverClockBiasKey Receiver clock bias node.
   * @param differentialCorrectionKey Differential correction node.
   * @param measuredPseudorange Receiver-measured pseudorange in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param satelliteClockBias Satellite clock bias in seconds.
   * @param model 1-D pseudorange noise model.
   */
  DifferentialPseudorangeFactor(
      Key receiverPositionKey, Key receiverClockBiasKey,
      Key differentialCorrectionKey, double measuredPseudorange,
      const Point3& satellitePosition, double satelliteClockBias = 0.0,
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
  Vector evaluateError(
      const Point3& receiverPosition, const double& receiverClock_bias,
      const double& differentialCorrection, OptionalMatrixType HreceiverPos,
      OptionalMatrixType HreceiverClockBias,
      OptionalMatrixType HdifferentialCorrection) const override;

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DifferentialPseudorangeFactor::Base);
    ar& BOOST_SERIALIZATION_NVP(pseudorange_);
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
 *   antenna_pos = ecef_T_body.translation() + ecef_R_body * bL_
 *
 * where bL_ is the lever arm from the body origin to the antenna in the body
 * frame. The error model is:
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

  Point3 bL_;  ///< Lever arm from body origin to antenna in body frame.
  std::optional<Pose3> ecef_T_nav_;  ///< Optional ECEF-from-nav transform.

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<PseudorangeFactorArm> shared_ptr;

  /// Typedef to this class
  typedef PseudorangeFactorArm This;

  /** default constructor - only use for serialization */
  PseudorangeFactorArm() : PseudorangeBase{0.0, Point3(0, 0, 0), 0.0}, bL_(0, 0, 0) {}

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
  inline const Point3& leverArm() const { return bL_; }

  /// return the optional ecef_T_nav transform
  inline const std::optional<Pose3>& ecefTnav() const { return ecef_T_nav_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(PseudorangeFactorArm::Base);
    ar& BOOST_SERIALIZATION_NVP(pseudorange_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
    ar& BOOST_SERIALIZATION_NVP(bL_);
    ar& BOOST_SERIALIZATION_NVP(ecef_T_nav_);
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

  Point3 bL_;  ///< Lever arm from body origin to antenna in body frame.
  std::optional<Pose3> ecef_T_nav_;  ///< Optional ECEF-from-nav transform.

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<DifferentialPseudorangeFactorArm> shared_ptr;

  /// Typedef to this class
  typedef DifferentialPseudorangeFactorArm This;

  /** default constructor - only use for serialization */
  DifferentialPseudorangeFactorArm() : PseudorangeBase{0.0, Point3(0, 0, 0), 0.0}, bL_(0, 0, 0) {}

  virtual ~DifferentialPseudorangeFactorArm() = default;

  /**
   * Construct a DifferentialPseudorangeFactorArm (ECEF pose key).
   *
   * @param poseKey Receiver gtsam::Pose3 key (body pose in ECEF frame).
   * @param receiverClockBiasKey Receiver clock bias node.
   * @param differentialCorrectionKey Differential correction node.
   * @param measuredPseudorange Receiver-measured pseudorange in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param leverArm Translation from body origin to antenna in body frame.
   * @param satelliteClockBias Satellite clock bias in seconds.
   * @param model 1-D pseudorange noise model.
   */
  DifferentialPseudorangeFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key differentialCorrectionKey,
      double measuredPseudorange, const Point3& satellitePosition,
      const Point3& leverArm, double satelliteClockBias = 0.0,
      const SharedNoiseModel& model = noiseModel::Unit::Create(1));

  /**
   * Construct a DifferentialPseudorangeFactorArm with ecef_T_nav.
   *
   * @param poseKey Receiver gtsam::Pose3 key (body pose in local nav frame).
   * @param receiverClockBiasKey Receiver clock bias node.
   * @param differentialCorrectionKey Differential correction node.
   * @param measuredPseudorange Receiver-measured pseudorange in meters.
   * @param satellitePosition Satellite ECEF position in meters.
   * @param leverArm Translation from body origin to antenna in body frame.
   * @param ecef_T_nav Transform from local navigation frame to ECEF.
   * @param satelliteClockBias Satellite clock bias in seconds.
   * @param model 1-D pseudorange noise model.
   */
  DifferentialPseudorangeFactorArm(
      Key poseKey, Key receiverClockBiasKey, Key differentialCorrectionKey,
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
                       const double& differentialCorrection,
                       OptionalMatrixType H_pose,
                       OptionalMatrixType HreceiverClockBias,
                       OptionalMatrixType HdifferentialCorrection) const override;

  /// return the lever arm, a position in the body frame
  inline const Point3& leverArm() const { return bL_; }

  /// return the optional ecef_T_nav transform
  inline const std::optional<Pose3>& ecefTnav() const { return ecef_T_nav_; }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION  ///
  /// Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(
        DifferentialPseudorangeFactorArm::Base);
    ar& BOOST_SERIALIZATION_NVP(pseudorange_);
    ar& BOOST_SERIALIZATION_NVP(satPos_);
    ar& BOOST_SERIALIZATION_NVP(satClkBias_);
    ar& BOOST_SERIALIZATION_NVP(bL_);
    ar& BOOST_SERIALIZATION_NVP(ecef_T_nav_);
  }
#endif
};

/// traits
template <>
struct traits<DifferentialPseudorangeFactorArm>
    : public Testable<DifferentialPseudorangeFactorArm> {};

}  // namespace gtsam
