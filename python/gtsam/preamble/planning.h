// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Akash Ratheesh and Frank Dellaert.
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/planning/MPPITracking.h>

namespace gtsam::python {
namespace py = pybind11;

/// Invoke a Python discrete step and convert its result to the planner state.
struct PythonStep {
  py::function function;

  template <class State, class Control>
  State operator()(const State& state, const Control& control) const {
    py::gil_scoped_acquire acquire;
    return function(state, control).template cast<State>();
  }
};

/// Bind polymorphic SamplingAlgorithm search overloads for a proposal type.
template <class Dynamics, class Proposal, class Planner>
void bindSearch(py::class_<Planner, std::shared_ptr<Planner>>& cls) {
  using Release = py::call_guard<py::gil_scoped_release>;
  cls.def(
         "iterate",
         [](const Planner& planner, const NonlinearFactorGraph& cost,
            const Values& initial, const Dynamics& dynamics,
            const Proposal& proposal, const SamplingAlgorithm& rule,
            typename Proposal::State& search) {
           planner.iterate(cost, initial, dynamics, proposal, rule, search);
         },
         py::arg("cost"), py::arg("initial"), py::arg("dynamics"),
         py::arg("proposal"), py::arg("algorithm"), py::arg("search"),
         Release())
      .def("optimize",
           &Planner::template optimize<Dynamics, Proposal, SamplingAlgorithm>,
           py::arg("cost"), py::arg("initial"), py::arg("dynamics"),
           py::arg("proposal"), py::arg("algorithm"), py::arg("iterations"),
           Release());
}

/// Release the GIL around rollouts. Python callbacks reacquire it at
/// invocation.
template <class Dynamics, class Planner>
void bindPythonDynamics(py::class_<Planner, std::shared_ptr<Planner>>& cls) {
  using Release = py::call_guard<py::gil_scoped_release>;
  cls.def("rollout", &Planner::template rollout<Dynamics>, py::arg("initial"),
          py::arg("controls"), py::arg("dynamics"), Release())
      .def("rolloutAndScore", &Planner::template rolloutAndScore<Dynamics>,
           py::arg("cost"), py::arg("initial"), py::arg("controls"),
           py::arg("dynamics"), Release());
  // Colored, normal-lognormal, and piecewise-linear proposals use
  // GaussianProposal.
  bindSearch<Dynamics, GaussianProposal>(cls);
  bindSearch<Dynamics, SmoothControlProposal>(cls);
}

template <class State, class Control>
void bindPlannerCallbacks(py::module_& module, const char* name) {
  using Planner = SamplingPlanner<State, Control>;
  using Field = std::function<typename traits<State>::TangentVector(
      const State&, const Control&)>;
  auto cls =
      py::reinterpret_borrow<py::class_<Planner, std::shared_ptr<Planner>>>(
          module.attr(name));
  using Policy = std::function<Control(size_t, const State&)>;
  cls.def("rolloutPolicy", &Planner::template rolloutPolicy<Policy, Field>,
          py::arg("initial"), py::arg("policy"), py::arg("dynamics"),
          py::call_guard<py::gil_scoped_release>());
  bindPythonDynamics<Field>(cls);
  bindPythonDynamics<DiscreteTransition<PythonStep>>(cls);
}

/// Bind proposal-parameterized tube and robust tracking overloads.
template <class State, class Dynamics, class Feedback, class Proposal>
void bindTrackingProposal(
    py::class_<MPPITracking, std::shared_ptr<MPPITracking>>& cls) {
  using Release = py::call_guard<py::gil_scoped_release>;
  cls.def("tube",
          &MPPITracking::tube<State, Vector, Dynamics, Feedback, Proposal>,
          py::arg("planner"), py::arg("graph"), py::arg("nominal"),
          py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
          py::arg("proposal"), py::arg("threshold"), py::arg("iterations"),
          Release())
      .def("robust",
           &MPPITracking::robust<State, Vector, Dynamics, Feedback, Proposal>,
           py::arg("planner"), py::arg("graph"), py::arg("previousNominal"),
           py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
           py::arg("proposal"), py::arg("threshold"), py::arg("candidates"),
           py::arg("stride"), py::arg("iterations"),
           py::arg("feedbackPrecision"), Release());
}

/// Bind MPPITracking's templated Python callback methods for a state type.
template <class State>
void bindTrackingCallbacks(py::module_& module) {
  using Release = py::call_guard<py::gil_scoped_release>;
  using Dynamics = std::function<typename traits<State>::TangentVector(
      const State&, const Vector&)>;
  using Feedback =
      std::function<Vector(size_t, const State&, const State&, const Vector&)>;
  auto cls = py::reinterpret_borrow<
      py::class_<MPPITracking, std::shared_ptr<MPPITracking>>>(
      module.attr("MPPITracking"));
  cls.def("colored", &MPPITracking::colored<State, Vector, Dynamics>,
          py::arg("planner"), py::arg("graph"), py::arg("initial"),
          py::arg("dynamics"), py::arg("exponent"), py::arg("iterations"),
          py::arg("tsallisCutoff") = 0.0, py::arg("tsallisShape") = 2.0,
          Release())
      .def("tube", &MPPITracking::tube<State, Vector, Dynamics, Feedback>,
           py::arg("planner"), py::arg("graph"), py::arg("nominal"),
           py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
           py::arg("threshold"), py::arg("iterations"), Release())
      .def("robust", &MPPITracking::robust<State, Vector, Dynamics, Feedback>,
           py::arg("planner"), py::arg("graph"), py::arg("previousNominal"),
           py::arg("actual"), py::arg("dynamics"), py::arg("feedback"),
           py::arg("threshold"), py::arg("candidates"), py::arg("stride"),
           py::arg("iterations"), py::arg("feedbackPrecision"), Release())
      .def("primitives", &MPPITracking::primitives<State, Vector, Dynamics>,
           py::arg("planner"), py::arg("graph"), py::arg("initial"),
           py::arg("previousMPPI"), py::arg("dynamics"), py::arg("segments"),
           py::arg("exponent"), py::arg("hysteresis"),
           py::arg("primitiveIterations"), py::arg("mppiIterations"),
           py::arg("stopThreshold"), py::arg("stoppingControls"),
           py::arg("tsallisCutoff") = 0.0, py::arg("tsallisShape") = 2.0,
           py::arg("mixture") = MotionPrimitiveOptions{}, Release());
  // Colored, normal-lognormal, and piecewise-linear proposals use
  // GaussianProposal.
  bindTrackingProposal<State, Dynamics, Feedback, GaussianProposal>(cls);
  bindTrackingProposal<State, Dynamics, Feedback, SmoothControlProposal>(cls);
}
}  // namespace gtsam::python
