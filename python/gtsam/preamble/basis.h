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

// We'll allow transparent binding of python dict to Sequence in this
// compilation unit using pybind11/stl.h.
// Another alternative would be making Sequence opaque in
// python/gtsam/{preamble, specializations}, but std::map<double, double> is
// common enough that it may cause collisions, and we don't need
// reference-access anyway.
#include <pybind11/stl.h>
