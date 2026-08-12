#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "gtsam/geometry/Pose3.h"

#include <type_traits>

namespace gtwrap {
namespace internal {

template <typename T>
struct PyArgPolicy {
  static pybind11::arg make(const char* name) { return pybind11::arg(name); }
};

template <typename T>
pybind11::arg py_arg(const char* name) {
  return PyArgPolicy<typename std::decay<T>::type>::make(name);
}

}  // namespace internal
}  // namespace gtwrap




using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(operator_py, m_) {
    m_.doc() = "pybind11 wrapper of operator_py";

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::Pose3, std::shared_ptr<gtsam::Pose3>>(m_gtsam, "Pose3")
        .def(py::init<>())
        .def(py::init<gtsam::Rot3, gtsam::Point3>(), gtwrap::internal::py_arg<gtsam::Rot3>("R"), gtwrap::internal::py_arg<gtsam::Point3>("t"))
        .def(py::self * py::self);

    py::class_<gtsam::Container<gtsam::Matrix>, std::shared_ptr<gtsam::Container<gtsam::Matrix>>>(m_gtsam, "ContainerMatrix")
        .def("__call__", &gtsam::Container<gtsam::Matrix>::operator())
        .def("__getitem__", &gtsam::Container<gtsam::Matrix>::operator[]);


#include "python/specializations.h"

}

