#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.





using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(templates_py, m_) {
    m_.doc() = "pybind11 wrapper of templates_py";


    py::class_<TemplatedConstructor, std::shared_ptr<TemplatedConstructor>>(m_, "TemplatedConstructor")
        .def(py::init<>())
        .def(py::init<const string&>(), py::arg("arg"))
        .def(py::init<const int&>(), py::arg("arg"))
        .def(py::init<const double&>(), py::arg("arg"));

    py::class_<ScopedTemplate<Result>, std::shared_ptr<ScopedTemplate<Result>>>(m_, "ScopedTemplateResult")
        .def(py::init<const Result::Value&>(), py::arg("arg"));


#include "python/specializations.h"

}

