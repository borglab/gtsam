// Authors: Frank Dellaert and Akash Ratheesh.
m_.attr("GaussianProposalState") = m_.attr("ControlDistribution");

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

gtsam::python::bindTrackingCallbacks<gtsam::Vector>(m_);
gtsam::python::bindTrackingCallbacks<gtsam::Pose2>(m_);
