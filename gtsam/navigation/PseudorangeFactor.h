/**
 *  @file   PseudorangeFactor.h
 *  @author Sammy Guo
 *  @brief  Header file for GNSS Pseudorange factor
 *  @date   January 18, 2026
 **/
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <string>

namespace gtsam {

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
class GTSAM_EXPORT PseudorangeFactor
    : public NoiseModelFactorN<Point3, double> {
 private:
  typedef NoiseModelFactorN<Point3, double> Base;

  double
      pseudorange_;    ///< Receiver-reported pseudorange measurement in meters.
  Point3 satPos_;      ///< Satellite position in WGS84 ECEF meters.
  double satClkBias_;  ///< Satellite clock bias in seconds.

 public:
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;

  /// shorthand for a smart pointer to a factor
  typedef std::shared_ptr<PseudorangeFactor> shared_ptr;

  /// Typedef to this class
  typedef PseudorangeFactor This;

  /** default constructor - only use for serialization */
  PseudorangeFactor();

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
   * @param model 1-D noise model.
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
                       const double& receiverClock_bias,
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

}  // namespace gtsam
