/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   SmartProjectionFactor.h
 * @brief  Smart factor on cameras (pose + calibration)
 * @author Luca Carlone
 * @author Zsolt Kira
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/slam/SmartProjectionFactorBase.h>

namespace gtsam {

/**
 * Smart factor for monocular cameras whose pose and calibration are variables.
 * Values must contain the involved cameras. When calibration is fixed, use
 * SmartProjectionPoseFactor instead.
 */
template <class CAMERA>
class SmartProjectionFactor : public SmartProjectionFactorBase<CAMERA> {
 private:
  using Base = SmartProjectionFactorBase<CAMERA>;
  using This = SmartProjectionFactor<CAMERA>;

 public:
  /// Shorthand for a smart pointer to this factor.
  using shared_ptr = std::shared_ptr<This>;

  /// Default constructor, only for serialization.
  SmartProjectionFactor() = default;

  /** Construct from an isotropic measurement noise model and parameters. */
  SmartProjectionFactor(
      const SharedNoiseModel& sharedNoiseModel,
      const SmartProjectionParams& params = SmartProjectionParams())
      : Base(sharedNoiseModel, params) {}

  /// Compare with another camera-variable smart projection factor.
  bool equals(const NonlinearFactor& factor, double tol = 1e-9) const override {
    const auto* other = dynamic_cast<const This*>(&factor);
    return other && Base::equals(factor, tol);
  }

 private:
#if GTSAM_ENABLE_BOOST_SERIALIZATION
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& archive, const unsigned int /*version*/) {
    archive& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
#endif
};

/// Testable traits for SmartProjectionFactor.
template <class CAMERA>
struct traits<SmartProjectionFactor<CAMERA>>
    : public Testable<SmartProjectionFactor<CAMERA>> {};

}  // namespace gtsam
