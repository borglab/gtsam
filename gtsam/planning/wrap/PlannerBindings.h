// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Akash Ratheesh and Frank Dellaert.
#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/planning/SamplingPlanner.h>

#include <functional>

namespace gtsam {

// Named callable types let MATLAB and Python pass compiled dynamics to the
// same templated C++ planner. The dynamics themselves belong to the caller.
struct Vector4Point2Field {
  using Function = Vector4(const Vector4&, const Point2&);
  Function& function;

  Vector4 operator()(const Vector4& state, const Point2& control) const {
    return function(state, control);
  }
};

using Pose2Point2Transition =
    DiscreteTransition<std::function<Pose2(const Pose2&, const Point2&)>>;

// The third parameter selects the callable overload emitted by the wrapper.
template <class State, class Control, class Dynamics>
using WrappedSamplingPlanner = SamplingPlanner<State, Control>;

}  // namespace gtsam
