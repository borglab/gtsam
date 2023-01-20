/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   PinholeFactor.h
 *  @brief  helper class for tests
 *  @author Frank Dellaert
 *  @date   February 2022
 */

#pragma once

namespace gtsam {
template <typename T>
struct traits;
}

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/SmartFactorBase.h>

namespace gtsam {

class PinholeFactor : public SmartFactorBase<PinholeCamera<Cal3Bundler> > {
 public:
  typedef SmartFactorBase<PinholeCamera<Cal3Bundler> > Base;
  PinholeFactor() {}
  PinholeFactor(const SharedNoiseModel& sharedNoiseModel,
                boost::optional<Pose3> body_P_sensor = boost::none,
                size_t expectedNumberCameras = 10)
      : Base(sharedNoiseModel, body_P_sensor, expectedNumberCameras) {}
  double error(const Values& values) const override { return 0.0; }
  boost::shared_ptr<GaussianFactor> linearize(
      const Values& values) const override {
    return boost::shared_ptr<GaussianFactor>(new JacobianFactor());
  }
};

/// traits
template <>
struct traits<PinholeFactor> : public Testable<PinholeFactor> {};

}  // namespace gtsam
