// Authors: Frank Dellaert and Akash Ratheesh.
using GaussianState = gtsam::GaussianProposal::State;
using SmoothState = gtsam::SmoothControlProposal::State;
m_.attr("GaussianProposalState") = m_.attr("ControlDistribution");
py::class_<SmoothState, GaussianState, std::shared_ptr<SmoothState>>(m_, "SmoothControlProposalState")
    .def_readwrite("derivativeMean", &SmoothState::derivativeMean);

py::reinterpret_borrow<py::class_<gtsam::SmoothControlProposal, std::shared_ptr<gtsam::SmoothControlProposal>>>(
    m_.attr("SmoothControlProposal"))
    .def("initialize", &gtsam::SmoothControlProposal::initialize, py::arg("nominal"));
#include "python/gtsam/specializations/mppi_variants.h"

using gtsam::python::PythonStep;
py::class_<gtsam::DiscreteTransition<PythonStep>>(m_, "DiscreteTransition");
m_.def(
    "discreteTransition",
    [](py::function step) {
      return gtsam::discreteTransition(PythonStep{std::move(step)}, false);
    },
    py::arg("step"),
    "Mark step(state, control) as a discrete next-state function.");

gtsam::python::bindPlannerCallbacks<gtsam::Vector, gtsam::Vector>(
    m_, "SamplingPlannerVectorVector");
gtsam::python::bindPlannerCallbacks<gtsam::Pose2, gtsam::Vector>(
    m_, "SamplingPlannerPose2Vector");
gtsam::python::bindPlannerCallbacks<gtsam::Pose3, gtsam::Vector>(
    m_, "SamplingPlannerPose3Vector");
