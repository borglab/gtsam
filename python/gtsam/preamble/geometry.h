/* Please refer to:
 * https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
 * These are required to save one copy operation on Python calls.
 *
 * NOTES
 * =================
 *
 * `PYBIND11_MAKE_OPAQUE` will mark the type as "opaque" for the pybind11
 * automatic STL binding, such that the raw objects can be accessed in Python.
 * Without this they will be automatically converted to a Python object, and all
 * mutations on Python side will not be reflected on C++.
 */

// Support for binding boost::optional types in C++11.
// https://pybind11.readthedocs.io/en/stable/advanced/cast/stl.html
namespace pybind11 { namespace detail {
    template <typename T>
    struct type_caster<boost::optional<T>> : optional_caster<boost::optional<T>> {};
}}

PYBIND11_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3_S2>>);
PYBIND11_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Bundler>>);
PYBIND11_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Unified>>);
PYBIND11_MAKE_OPAQUE(gtsam::CameraSet<gtsam::PinholeCamera<gtsam::Cal3Fisheye>>);
