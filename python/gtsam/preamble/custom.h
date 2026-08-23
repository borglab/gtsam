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

// JacobianVector (std::vector<gtsam::Matrix>) is made opaque in
// python/gtsam/optional_jacobian_pybind.h, which both module templates include,
// so that it is opaque in every translation unit rather than only this one.
