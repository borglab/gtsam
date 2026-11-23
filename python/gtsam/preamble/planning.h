// Copyright (c) Georgia Tech Research Corporation. See LICENSE for details.
// Authors: Akash Ratheesh and Frank Dellaert.
#pragma once

#include <gtsam/planning/SamplingPlanner.h>
#include <pybind11/functional.h>

namespace gtsam::python {
namespace py = pybind11;

struct PythonDiscreteTransition {
  py::function step;
};

// Convert a callback while holding the GIL, then release it during the solve.
// pybind11 reacquires the GIL when a worker calls a Python vector field.
template <class Input, class Planner, class Convert>
void bindPlanningCallback(py::class_<Planner, std::shared_ptr<Planner>>& cls,
                          Convert convert) {
  const auto withDynamics = [convert](const Input& callback, auto&& run) {
    auto dynamics = convert(callback);
    py::gil_scoped_release release;
    return run(dynamics);
  };
  cls.def(
      "rollout",
      [withDynamics](const Planner& planner, const Values& initial,
                     const Matrix& controls, const Input& dynamics) {
        return withDynamics(dynamics, [&](const auto& model) {
          return planner.rollout(initial, controls, model);
        });
      },
      py::arg("initial"), py::arg("controls"), py::arg("dynamics"));
  cls.def(
      "rolloutAndScore",
      [withDynamics](const Planner& planner, const NonlinearFactorGraph& graph,
                     const Values& initial, const Matrix& controls,
                     const Input& dynamics) {
        return withDynamics(dynamics, [&](const auto& model) {
          return planner.rolloutAndScore(graph, initial, controls, model);
        });
      },
      py::arg("cost"), py::arg("initial"), py::arg("controls"),
      py::arg("dynamics"));
  cls.def(
      "iterate",
      [withDynamics](const Planner& planner, const NonlinearFactorGraph& graph,
                     const Values& initial, const Input& dynamics,
                     const GaussianProposal& proposal, const MPPI& algorithm,
                     ControlDistribution& search) {
        withDynamics(dynamics, [&](const auto& model) {
          planner.iterate(graph, initial, model, proposal, algorithm, search);
        });
      },
      py::arg("cost"), py::arg("initial"), py::arg("dynamics"),
      py::arg("proposal"), py::arg("algorithm"), py::arg("search"));
  cls.def(
      "optimize",
      [withDynamics](const Planner& planner, const NonlinearFactorGraph& graph,
                     const Values& initial, const Input& dynamics,
                     const GaussianProposal& proposal, const MPPI& algorithm,
                     size_t iterations) {
        return withDynamics(dynamics, [&](const auto& model) {
          return planner.optimize(graph, initial, model, proposal, algorithm,
                                  iterations);
        });
      },
      py::arg("cost"), py::arg("initial"), py::arg("dynamics"),
      py::arg("proposal"), py::arg("algorithm"), py::arg("iterations"));
}

template <class State, class Control>
void bindPlannerCallbacks(py::module_& module, const char* name) {
  using Planner = SamplingPlanner<State, Control>;
  using Field = std::function<typename traits<State>::TangentVector(
      const State&, const Control&)>;
  using Step = std::function<State(const State&, const Control&)>;
  auto cls =
      py::reinterpret_borrow<py::class_<Planner, std::shared_ptr<Planner>>>(
          module.attr(name));
  bindPlanningCallback<Field>(cls, [](const Field& field) { return field; });
  bindPlanningCallback<PythonDiscreteTransition>(
      cls, [](const PythonDiscreteTransition& model) {
        return discreteTransition(model.step.cast<Step>(), false);
      });
}
}  // namespace gtsam::python
