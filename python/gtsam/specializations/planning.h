// Authors: Akash Ratheesh and Frank Dellaert. See LICENSE for details.
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
