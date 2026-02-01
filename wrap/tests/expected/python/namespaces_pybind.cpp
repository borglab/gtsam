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




using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(namespaces_py, m_) {
    m_.doc() = "pybind11 wrapper of namespaces_py";

    pybind11::module m_ns1 = m_.def_submodule("ns1", "ns1 submodule");

    py::class_<ns1::ClassA, std::shared_ptr<ns1::ClassA>>(m_ns1, "ClassA")
        .def(py::init<>());

    py::class_<ns1::ClassB, std::shared_ptr<ns1::ClassB>>(m_ns1, "ClassB")
        .def(py::init<>());

    m_ns1.def("aGlobalFunction",[](){return ns1::aGlobalFunction();});    pybind11::module m_ns2 = m_.def_submodule("ns2", "ns2 submodule");

    py::class_<ns2::ClassA, std::shared_ptr<ns2::ClassA>>(m_ns2, "ClassA")
        .def(py::init<>())
        .def("memberFunction",[](ns2::ClassA* self){return self->memberFunction();})
        .def("nsArg",[](ns2::ClassA* self, const ns1::ClassB& arg){return self->nsArg(arg);}, py::arg("arg"))
        .def("nsReturn",[](ns2::ClassA* self, double q){return self->nsReturn(q);}, py::arg("q"))
        .def_static("afunction",[](){return ns2::ClassA::afunction();});
    pybind11::module m_ns2_ns3 = m_ns2.def_submodule("ns3", "ns3 submodule");

    py::class_<ns2::ns3::ClassB, std::shared_ptr<ns2::ns3::ClassB>>(m_ns2_ns3, "ClassB")
        .def(py::init<>());

    py::class_<ns2::ClassC, std::shared_ptr<ns2::ClassC>>(m_ns2, "ClassC")
        .def(py::init<>());

    m_ns2.attr("aNs2Var") = ns2::aNs2Var;
    m_ns2.def("aGlobalFunction",[](){return ns2::aGlobalFunction();});
    m_ns2.def("overloadedGlobalFunction",[](const ns1::ClassA& a){return ns2::overloadedGlobalFunction(a);}, py::arg("a"));
    m_ns2.def("overloadedGlobalFunction",[](const ns1::ClassA& a, double b){return ns2::overloadedGlobalFunction(a, b);}, py::arg("a"), py::arg("b"));
    py::class_<ClassD, std::shared_ptr<ClassD>>(m_, "ClassD")
        .def(py::init<>());

    m_.attr("aGlobalVar") = aGlobalVar;    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::Values, std::shared_ptr<gtsam::Values>>(m_gtsam, "Values")
        .def(py::init<>())
        .def(py::init<const gtsam::Values&>(), py::arg("other"))
        .def("insert_vector",[](gtsam::Values* self, size_t j, const gtsam::Vector& vector){ self->insert(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Vector& vector){ self->insert(j, vector);}, py::arg("j"), py::arg("vector"))
        .def("insert_matrix",[](gtsam::Values* self, size_t j, const gtsam::Matrix& matrix){ self->insert(j, matrix);}, py::arg("j"), py::arg("matrix"))
        .def("insert",[](gtsam::Values* self, size_t j, const gtsam::Matrix& matrix){ self->insert(j, matrix);}, py::arg("j"), py::arg("matrix"));


#include "python/specializations.h"

}

