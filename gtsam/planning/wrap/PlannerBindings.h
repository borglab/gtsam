// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Akash Ratheesh and Frank Dellaert.
#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/planning/SamplingPlanner.h>

#include <functional>

namespace gtsam {

// Named callable types let MATLAB and Python pass compiled dynamics to the
// same templated C++ planner. The dynamics themselves belong to the caller.
struct VectorField {
  using Function = Vector(const Vector&, const Vector&);
  Function& function;

  Vector operator()(const Vector& state, const Vector& control) const {
    return function(state, control);
  }
};

using Pose2Transition =
    DiscreteTransition<std::function<Pose2(const Pose2&, const Vector&)>>;

}  // namespace gtsam
