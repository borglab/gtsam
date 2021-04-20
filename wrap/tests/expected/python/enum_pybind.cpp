

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.


#include "wrap/serialization.h"
#include <boost/serialization/export.hpp>





using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(enum_py, m_) {
    m_.doc() = "pybind11 wrapper of enum_py";


    py::enum_<Kind>(m_, "Kind", py::arithmetic())
        .value("Dog", Kind::Dog)
        .value("Cat", Kind::Cat);

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::enum_<gtsam::VerbosityLM>(m_gtsam, "VerbosityLM", py::arithmetic())
        .value("SILENT", gtsam::VerbosityLM::SILENT)
        .value("SUMMARY", gtsam::VerbosityLM::SUMMARY)
        .value("TERMINATION", gtsam::VerbosityLM::TERMINATION)
        .value("LAMBDA", gtsam::VerbosityLM::LAMBDA)
        .value("TRYLAMBDA", gtsam::VerbosityLM::TRYLAMBDA)
        .value("TRYCONFIG", gtsam::VerbosityLM::TRYCONFIG)
        .value("DAMPED", gtsam::VerbosityLM::DAMPED)
        .value("TRYDELTA", gtsam::VerbosityLM::TRYDELTA);


    py::class_<gtsam::Pet, std::shared_ptr<gtsam::Pet>>(m_gtsam, "Pet")
        .def(py::init<const string&, Kind>(), py::arg("name"), py::arg("type"))
        .def_readwrite("name", &gtsam::Pet::name)
        .def_readwrite("type", &gtsam::Pet::type);


#include "python/specializations.h"

}

