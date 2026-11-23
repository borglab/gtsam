// Authors: Akash Ratheesh and Frank Dellaert. See LICENSE for details.
using gtsam::python::PythonDiscreteTransition;
py::class_<PythonDiscreteTransition>(m_, "DiscreteTransition");
m_.def(
    "discreteTransition",
    [](py::function step) { return PythonDiscreteTransition{std::move(step)}; },
    py::arg("step"),
    "Mark step(state, control) as a discrete next-state function.");

gtsam::python::bindPlannerCallbacks<gtsam::Vector, gtsam::Vector>(
    m_, "SamplingPlannerVectorVector");
gtsam::python::bindPlannerCallbacks<gtsam::Vector4, gtsam::Point2>(
    m_, "SamplingPlannerVector4Point2");
gtsam::python::bindPlannerCallbacks<gtsam::Pose2, gtsam::Point2>(
    m_, "SamplingPlannerPose2Point2");
