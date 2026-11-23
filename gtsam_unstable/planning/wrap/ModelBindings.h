// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Akash Ratheesh and Frank Dellaert.
#pragma once

#include <gtsam/planning/wrap/PlannerBindings.h>
#include <gtsam_unstable/planning/dynamics/models/PointMass.h>
#include <gtsam_unstable/planning/dynamics/models/Unicycle.h>

namespace gtsam {

/// The point mass supplies a tangent vector. The planner applies its own dt.
inline VectorField pointMassVectorField() { return {PointMass::dynamics}; }

/// The unicycle supplies a complete discrete step using its model time step.
inline Pose2Transition unicycleTransition(const Unicycle& model) {
  return {
      [model](const Pose2& state, const Vector& control) {
        if (control.size() != 2)
          throw std::invalid_argument("Unicycle expects speed and turn rate.");
        return model.predict(state, control);
      },
      true};
}

}  // namespace gtsam
