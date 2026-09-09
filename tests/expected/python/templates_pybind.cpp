#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.


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



void gtwrap_declare_templates_py(py::module_ &m_) {

    py::class_<TemplatedConstructor, std::shared_ptr<TemplatedConstructor>>(m_, "TemplatedConstructor");

    py::class_<ScopedTemplate<Result>, std::shared_ptr<ScopedTemplate<Result>>>(m_, "ScopedTemplateResult");

}

void gtwrap_bind_templates_py(py::module_ &m_) {
#include "python/specializations.h"

    auto gtwrap_class_m__TemplatedConstructor = py::reinterpret_borrow<py::class_<TemplatedConstructor, std::shared_ptr<TemplatedConstructor>>>(m_.attr("TemplatedConstructor"));
    gtwrap_class_m__TemplatedConstructor
        .def(py::init<>())
        .def(py::init<const string&>(), gtwrap::internal::py_arg<const string&>("arg"))
        .def(py::init<const int&>(), gtwrap::internal::py_arg<const int&>("arg"))
        .def(py::init<const double&>(), gtwrap::internal::py_arg<const double&>("arg"));

    auto gtwrap_class_m__ScopedTemplateResult = py::reinterpret_borrow<py::class_<ScopedTemplate<Result>, std::shared_ptr<ScopedTemplate<Result>>>>(m_.attr("ScopedTemplateResult"));
    gtwrap_class_m__ScopedTemplateResult
        .def(py::init<const Result::Value&>(), gtwrap::internal::py_arg<const Result::Value&>("arg"));

}

PYBIND11_MODULE(templates_py, m_) {
    m_.doc() = "pybind11 wrapper of templates_py";

gtwrap_declare_templates_py(m_);
gtwrap_bind_templates_py(m_);
}
