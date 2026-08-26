#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "path/to/ns1.h"
#include "path/to/ns1/ClassB.h"
#include "path/to/ns2.h"
#include "path/to/ns2/ClassA.h"
#include "path/to/ns3.h"
#include "gtsam/nonlinear/Values.h"

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



void gtwrap_declare_namespaces_py(py::module_ &m_) {

    pybind11::module m_ns1 = m_.def_submodule("ns1", "ns1 submodule");

    py::class_<ns1::ClassA, std::shared_ptr<ns1::ClassA>>(m_ns1, "ClassA");

    py::class_<ns1::ClassB, std::shared_ptr<ns1::ClassB>>(m_ns1, "ClassB");

    pybind11::module m_ns2 = m_.def_submodule("ns2", "ns2 submodule");

    py::class_<ns2::ClassA, std::shared_ptr<ns2::ClassA>>(m_ns2, "ClassA");

    pybind11::module m_ns2_ns3 = m_ns2.def_submodule("ns3", "ns3 submodule");

    py::class_<ns2::ns3::ClassB, std::shared_ptr<ns2::ns3::ClassB>>(m_ns2_ns3, "ClassB");

    py::class_<ns2::ClassC, std::shared_ptr<ns2::ClassC>>(m_ns2, "ClassC");

    py::class_<ClassD, std::shared_ptr<ClassD>>(m_, "ClassD");

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::Values, std::shared_ptr<gtsam::Values>>(m_gtsam, "Values");

}

void gtwrap_bind_namespaces_py(py::module_ &m_) {
#include "python/specializations.h"

    pybind11::module m_ns1 = py::reinterpret_borrow<pybind11::module>(m_.attr("ns1"));

    auto gtwrap_class_m_ns1_ClassA = py::reinterpret_borrow<py::class_<ns1::ClassA, std::shared_ptr<ns1::ClassA>>>(m_ns1.attr("ClassA"));
    gtwrap_class_m_ns1_ClassA
        .def(py::init<>());

    auto gtwrap_class_m_ns1_ClassB = py::reinterpret_borrow<py::class_<ns1::ClassB, std::shared_ptr<ns1::ClassB>>>(m_ns1.attr("ClassB"));
    gtwrap_class_m_ns1_ClassB
        .def(py::init<>());

    m_ns1.def("aGlobalFunction",static_cast<gtsam::Vector (*)()>(&ns1::aGlobalFunction));
    pybind11::module m_ns2 = py::reinterpret_borrow<pybind11::module>(m_.attr("ns2"));

    auto gtwrap_class_m_ns2_ClassA = py::reinterpret_borrow<py::class_<ns2::ClassA, std::shared_ptr<ns2::ClassA>>>(m_ns2.attr("ClassA"));
    gtwrap_class_m_ns2_ClassA
        .def(py::init<>())
        .def("memberFunction",static_cast<double (ns2::ClassA::*)()>(&ns2::ClassA::memberFunction))
        .def("nsArg",static_cast<int (ns2::ClassA::*)(const ns1::ClassB&)>(&ns2::ClassA::nsArg), gtwrap::internal::py_arg<const ns1::ClassB&>("arg"))
        .def("nsReturn",static_cast<ns2::ns3::ClassB (ns2::ClassA::*)(double)>(&ns2::ClassA::nsReturn), gtwrap::internal::py_arg<double>("q"))
        .def_static("afunction",static_cast<double (*)()>(&ns2::ClassA::afunction));

    pybind11::module m_ns2_ns3 = py::reinterpret_borrow<pybind11::module>(m_ns2.attr("ns3"));

    auto gtwrap_class_m_ns2_ns3_ClassB = py::reinterpret_borrow<py::class_<ns2::ns3::ClassB, std::shared_ptr<ns2::ns3::ClassB>>>(m_ns2_ns3.attr("ClassB"));
    gtwrap_class_m_ns2_ns3_ClassB
        .def(py::init<>());

    auto gtwrap_class_m_ns2_ClassC = py::reinterpret_borrow<py::class_<ns2::ClassC, std::shared_ptr<ns2::ClassC>>>(m_ns2.attr("ClassC"));
    gtwrap_class_m_ns2_ClassC
        .def(py::init<>());

    m_ns2.attr("aNs2Var") = ns2::aNs2Var;
    m_ns2.def("aGlobalFunction",static_cast<gtsam::Vector (*)()>(&ns2::aGlobalFunction));
    m_ns2.def("overloadedGlobalFunction",static_cast<ns1::ClassA (*)(const ns1::ClassA&)>(&ns2::overloadedGlobalFunction), gtwrap::internal::py_arg<const ns1::ClassA&>("a"));
    m_ns2.def("overloadedGlobalFunction",static_cast<ns1::ClassA (*)(const ns1::ClassA&, double)>(&ns2::overloadedGlobalFunction), gtwrap::internal::py_arg<const ns1::ClassA&>("a"), gtwrap::internal::py_arg<double>("b"));
    auto gtwrap_class_m__ClassD = py::reinterpret_borrow<py::class_<ClassD, std::shared_ptr<ClassD>>>(m_.attr("ClassD"));
    gtwrap_class_m__ClassD
        .def(py::init<>());

    m_.attr("aGlobalVar") = aGlobalVar;
    pybind11::module m_gtsam = py::reinterpret_borrow<pybind11::module>(m_.attr("gtsam"));

    auto gtwrap_class_m_gtsam_Values = py::reinterpret_borrow<py::class_<gtsam::Values, std::shared_ptr<gtsam::Values>>>(m_gtsam.attr("Values"));
    gtwrap_class_m_gtsam_Values
        .def(py::init<>())
        .def(py::init<const gtsam::Values&>(), gtwrap::internal::py_arg<const gtsam::Values&>("other"))
        .def("insert_vector",[](gtsam::Values* self, size_t j, const gtsam::Vector& vector){ self->insert(j, vector);}, gtwrap::internal::py_arg<size_t>("j"), gtwrap::internal::py_arg<const gtsam::Vector&>("vector"))
        .def("insert",static_cast<void (gtsam::Values::*)(size_t, const gtsam::Vector&)>(&gtsam::Values::insert), gtwrap::internal::py_arg<size_t>("j"), gtwrap::internal::py_arg<const gtsam::Vector&>("vector"))
        .def("insert_matrix",[](gtsam::Values* self, size_t j, const gtsam::Matrix& matrix){ self->insert(j, matrix);}, gtwrap::internal::py_arg<size_t>("j"), gtwrap::internal::py_arg<const gtsam::Matrix&>("matrix"))
        .def("insert",static_cast<void (gtsam::Values::*)(size_t, const gtsam::Matrix&)>(&gtsam::Values::insert), gtwrap::internal::py_arg<size_t>("j"), gtwrap::internal::py_arg<const gtsam::Matrix&>("matrix"));

}

PYBIND11_MODULE(namespaces_py, m_) {
    m_.doc() = "pybind11 wrapper of namespaces_py";

gtwrap_declare_namespaces_py(m_);
gtwrap_bind_namespaces_py(m_);
}
