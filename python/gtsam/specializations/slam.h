/* Please refer to:
 * https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
 * These are required to save one copy operation on Python calls.
 *
 * NOTES
 * =================
 *
 * `py::bind_vector` and similar machinery gives the std container a Python-like
 * interface, but without the `<pybind11/stl.h>` copying mechanism. Combined
 * with `PYBIND11_MAKE_OPAQUE` this allows the types to be modified with Python,
 * and saves one copy operation.
 */

py::bind_vector<
    std::vector<std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3>>>>(
    m_, "BetweenFactorPose3s");
py::bind_vector<
    std::vector<std::shared_ptr<gtsam::BetweenFactor<gtsam::Pose2>>>>(
    m_, "BetweenFactorPose2s");
py::bind_vector<gtsam::Rot3Vector>(m_, "Rot3Vector");
