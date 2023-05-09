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
    std::vector<gtsam::Point2, Eigen::aligned_allocator<gtsam::Point2>>>(
    m_, "Point2Vector");
py::bind_vector<std::vector<gtsam::Point2Pair>>(m_, "Point2Pairs");
py::bind_vector<std::vector<gtsam::Point3Pair>>(m_, "Point3Pairs");
py::bind_vector<std::vector<gtsam::Pose3Pair>>(m_, "Pose3Pairs");
py::bind_vector<std::vector<gtsam::Pose3>>(m_, "Pose3Vector");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>>>(
    m_, "CameraSetCal3_S2");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>(
    m_, "CameraSetCal3Bundler");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Unified>>>(
    m_, "CameraSetCal3Unified");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>>(
    m_, "CameraSetCal3Fisheye");
