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



void gtwrap_declare_enum_py(py::module_ &m_) {
    py::enum_<Color>(m_, "Color", py::arithmetic())
        .value("Red", Color::Red)
        .value("Green", Color::Green)
        .value("Blue", Color::Blue);


    py::class_<Pet, std::shared_ptr<Pet>> gtwrap_class_m__Pet(m_, "Pet");
    py::enum_<Pet::Kind>(gtwrap_class_m__Pet, "Kind", py::arithmetic())
        .value("Dog", Pet::Kind::Dog)
        .value("Cat", Pet::Kind::Cat);


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


    py::class_<gtsam::MCU, std::shared_ptr<gtsam::MCU>> gtwrap_class_m_gtsam_MCU(m_gtsam, "MCU");
    py::enum_<gtsam::MCU::Avengers>(gtwrap_class_m_gtsam_MCU, "Avengers", py::arithmetic())
        .value("CaptainAmerica", gtsam::MCU::Avengers::CaptainAmerica)
        .value("IronMan", gtsam::MCU::Avengers::IronMan)
        .value("Hulk", gtsam::MCU::Avengers::Hulk)
        .value("Hawkeye", gtsam::MCU::Avengers::Hawkeye)
        .value("Thor", gtsam::MCU::Avengers::Thor);


    py::enum_<gtsam::MCU::GotG>(gtwrap_class_m_gtsam_MCU, "GotG", py::arithmetic())
        .value("Starlord", gtsam::MCU::GotG::Starlord)
        .value("Gamorra", gtsam::MCU::GotG::Gamorra)
        .value("Rocket", gtsam::MCU::GotG::Rocket)
        .value("Drax", gtsam::MCU::GotG::Drax)
        .value("Groot", gtsam::MCU::GotG::Groot);


    py::class_<gtsam::Optimizer<gtsam::GaussNewtonParams>, std::shared_ptr<gtsam::Optimizer<gtsam::GaussNewtonParams>>> gtwrap_class_m_gtsam_OptimizerGaussNewtonParams(m_gtsam, "OptimizerGaussNewtonParams");
    py::enum_<gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity>(gtwrap_class_m_gtsam_OptimizerGaussNewtonParams, "Verbosity", py::arithmetic())
        .value("SILENT", gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity::SILENT)
        .value("SUMMARY", gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity::SUMMARY)
        .value("VERBOSE", gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity::VERBOSE);


}

void gtwrap_bind_enum_py(py::module_ &m_) {
#include "python/specializations.h"

    auto gtwrap_class_m__Pet = py::reinterpret_borrow<py::class_<Pet, std::shared_ptr<Pet>>>(m_.attr("Pet"));
    gtwrap_class_m__Pet
        .def(py::init<const string&, Pet::Kind>(), gtwrap::internal::py_arg<const string&>("name"), gtwrap::internal::py_arg<Pet::Kind>("type"))
        .def("setColor",static_cast<void (Pet::*)(const Color&)>(&Pet::setColor), gtwrap::internal::py_arg<const Color&>("color"))
        .def("getColor",static_cast<Color (Pet::*)() const>(&Pet::getColor))
        .def_readwrite("name", &Pet::name)
        .def_readwrite("type", &Pet::type);

    pybind11::module m_gtsam = py::reinterpret_borrow<pybind11::module>(m_.attr("gtsam"));

    auto gtwrap_class_m_gtsam_MCU = py::reinterpret_borrow<py::class_<gtsam::MCU, std::shared_ptr<gtsam::MCU>>>(m_gtsam.attr("MCU"));
    gtwrap_class_m_gtsam_MCU
        .def(py::init<>());

    auto gtwrap_class_m_gtsam_OptimizerGaussNewtonParams = py::reinterpret_borrow<py::class_<gtsam::Optimizer<gtsam::GaussNewtonParams>, std::shared_ptr<gtsam::Optimizer<gtsam::GaussNewtonParams>>>>(m_gtsam.attr("OptimizerGaussNewtonParams"));
    gtwrap_class_m_gtsam_OptimizerGaussNewtonParams
        .def(py::init<const Optimizer<gtsam::GaussNewtonParams>::Verbosity&>(), gtwrap::internal::py_arg<const Optimizer<gtsam::GaussNewtonParams>::Verbosity&>("verbosity"))
        .def("setVerbosity",static_cast<void (gtsam::Optimizer<gtsam::GaussNewtonParams>::*)(const Optimizer<gtsam::GaussNewtonParams>::Verbosity)>(&gtsam::Optimizer<gtsam::GaussNewtonParams>::setVerbosity), gtwrap::internal::py_arg<const Optimizer<gtsam::GaussNewtonParams>::Verbosity>("value"))
        .def("getVerbosity",static_cast<gtsam::Optimizer::Verbosity (gtsam::Optimizer<gtsam::GaussNewtonParams>::*)() const>(&gtsam::Optimizer<gtsam::GaussNewtonParams>::getVerbosity))
        .def("getVerbosity",static_cast<gtsam::VerbosityLM (gtsam::Optimizer<gtsam::GaussNewtonParams>::*)() const>(&gtsam::Optimizer<gtsam::GaussNewtonParams>::getVerbosity));

}

PYBIND11_MODULE(enum_py, m_) {
    m_.doc() = "pybind11 wrapper of enum_py";

gtwrap_declare_enum_py(m_);
gtwrap_bind_enum_py(m_);
}
