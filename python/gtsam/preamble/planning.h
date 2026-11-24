// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Akash Ratheesh and Frank Dellaert.
#pragma once

#include <gtsam/planning/SamplingPlanner.h>
#include <pybind11/functional.h>

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

/// Bind a search rule and a proposal to a Python dynamics callable.
template <class Dynamics, class Proposal, class Rule, class Planner>
void bindSearch(py::class_<Planner, std::shared_ptr<Planner>>& cls) {
  using Release = py::call_guard<py::gil_scoped_release>;
  cls.def(
         "iterate",
         [](const Planner& planner, const NonlinearFactorGraph& cost,
            const Values& initial, const Dynamics& dynamics,
            const Proposal& proposal, const Rule& rule,
            typename Proposal::State& search) {
           planner.iterate(cost, initial, dynamics, proposal, rule, search);
         },
         py::arg("cost"), py::arg("initial"), py::arg("dynamics"),
         py::arg("proposal"), py::arg("algorithm"), py::arg("search"),
         Release())
      .def("optimize", &Planner::template optimize<Dynamics, Proposal, Rule>,
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
  bindSearch<Dynamics, GaussianProposal, MPPI>(cls);
  bindSearch<Dynamics, GaussianProposal, CEM>(cls);
  bindSearch<Dynamics, GaussianProposal, PredictiveSampling>(cls);
}

template <class State, class Control>
void bindPlannerCallbacks(py::module_& module, const char* name) {
  using Planner = SamplingPlanner<State, Control>;
  using Field = std::function<typename traits<State>::TangentVector(
      const State&, const Control&)>;
  auto cls =
      py::reinterpret_borrow<py::class_<Planner, std::shared_ptr<Planner>>>(
          module.attr(name));
  bindPythonDynamics<Field>(cls);
  bindPythonDynamics<DiscreteTransition<PythonStep>>(cls);
}
}  // namespace gtsam::python
