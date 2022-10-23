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

// Including <stl.h> can cause some serious hard-to-debug bugs!!!
// #include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

PYBIND11_MAKE_OPAQUE(std::vector<gtsam::SfmMeasurement>);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::SfmTrack>);
PYBIND11_MAKE_OPAQUE(std::vector<gtsam::SfmCamera>);
PYBIND11_MAKE_OPAQUE(
    std::vector<gtsam::BinaryMeasurement<gtsam::Unit3>>);
PYBIND11_MAKE_OPAQUE(
    std::vector<gtsam::BinaryMeasurement<gtsam::Rot3>>);
PYBIND11_MAKE_OPAQUE(
    std::vector<gtsam::gtsfm::Keypoints>);
PYBIND11_MAKE_OPAQUE(gtsam::gtsfm::MatchIndicesMap);