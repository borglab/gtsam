/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    LinearPlanningFactor.h
 * @author  Frank Dellaert
 * @author  Akash Ratheesh
 */

#pragma once

#include <gtsam/base/VectorSpace.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

namespace planning_test {

/// Test dynamics for a planar position incremented by its control.
class LinearPlanningFactor
    : public gtsam::NoiseModelFactorT<gtsam::Vector2, gtsam::Vector2,
                                      gtsam::Vector2, gtsam::Vector2> {
 public:
  /// Constrain nextPosition to position + control.
  LinearPlanningFactor(gtsam::Key position, gtsam::Key control,
                       gtsam::Key nextPosition)
      : NoiseModelFactorT(gtsam::noiseModel::Unit::Create(2), position, control,
                          nextPosition) {}

  /// Integrate a position increment over one step.
  gtsam::Vector2 predict(const gtsam::Vector2& position,
                         const gtsam::Vector2& control) const {
    return position + control;
  }

  /// Difference between the predicted and supplied next positions.
  gtsam::Vector2 evaluateError(const gtsam::Vector2& position,
                               const gtsam::Vector2& control,
                               const gtsam::Vector2& nextPosition,
                               gtsam::OptionalMatrixType H1,
                               gtsam::OptionalMatrixType H2,
                               gtsam::OptionalMatrixType H3) const override {
    if (H1) *H1 = gtsam::Matrix2::Identity();
    if (H2) *H2 = gtsam::Matrix2::Identity();
    if (H3) *H3 = -gtsam::Matrix2::Identity();
    return predict(position, control) - nextPosition;
  }
};

}  // namespace planning_test
