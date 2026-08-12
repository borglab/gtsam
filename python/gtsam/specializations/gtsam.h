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

/*
 * Custom Pybind11 module for the Mersenne-Twister PRNG object
 * `std::mt19937_64`.
 * This can be invoked with `gtsam.MT19937()` and passed
 * wherever a rng pointer is expected.
 */
#include <random>
py::class_<std::mt19937_64>(m_, "MT19937")
    .def(py::init<>())
    .def(py::init<std::mt19937_64::result_type>())
    .def("__call__", &std::mt19937_64::operator());
