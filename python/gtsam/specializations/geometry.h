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

py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>>>(
    m_, "CameraSetCal3_S2");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3DS2>>>(
    m_, "CameraSetCal3DS2");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>(
    m_, "CameraSetCal3Bundler");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Unified>>>(
    m_, "CameraSetCal3Unified");
py::bind_vector<gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>>(
    m_, "CameraSetCal3Fisheye");
