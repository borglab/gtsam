#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.





using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(enum_py, m_) {
    m_.doc() = "pybind11 wrapper of enum_py";

    py::enum_<Color>(m_, "Color", py::arithmetic())
        .value("Red", Color::Red)
        .value("Green", Color::Green)
        .value("Blue", Color::Blue);


    py::class_<Pet, std::shared_ptr<Pet>> pet(m_, "Pet");
    pet
        .def(py::init<const string&, Pet::Kind>(), py::arg("name"), py::arg("type"))
        .def("setColor",[](Pet* self, const Color& color){ self->setColor(color);}, py::arg("color"))
        .def("getColor",[](Pet* self){return self->getColor();})
        .def_readwrite("name", &Pet::name)
        .def_readwrite("type", &Pet::type);

    py::enum_<Pet::Kind>(pet, "Kind", py::arithmetic())
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


    py::class_<gtsam::MCU, std::shared_ptr<gtsam::MCU>> mcu(m_gtsam, "MCU");
    mcu
        .def(py::init<>());

    py::enum_<gtsam::MCU::Avengers>(mcu, "Avengers", py::arithmetic())
        .value("CaptainAmerica", gtsam::MCU::Avengers::CaptainAmerica)
        .value("IronMan", gtsam::MCU::Avengers::IronMan)
        .value("Hulk", gtsam::MCU::Avengers::Hulk)
        .value("Hawkeye", gtsam::MCU::Avengers::Hawkeye)
        .value("Thor", gtsam::MCU::Avengers::Thor);


    py::enum_<gtsam::MCU::GotG>(mcu, "GotG", py::arithmetic())
        .value("Starlord", gtsam::MCU::GotG::Starlord)
        .value("Gamorra", gtsam::MCU::GotG::Gamorra)
        .value("Rocket", gtsam::MCU::GotG::Rocket)
        .value("Drax", gtsam::MCU::GotG::Drax)
        .value("Groot", gtsam::MCU::GotG::Groot);


    py::class_<gtsam::Optimizer<gtsam::GaussNewtonParams>, std::shared_ptr<gtsam::Optimizer<gtsam::GaussNewtonParams>>> optimizergaussnewtonparams(m_gtsam, "OptimizerGaussNewtonParams");
    optimizergaussnewtonparams
        .def(py::init<const Optimizer<gtsam::GaussNewtonParams>::Verbosity&>(), py::arg("verbosity"))
        .def("setVerbosity",[](gtsam::Optimizer<gtsam::GaussNewtonParams>* self, const Optimizer<gtsam::GaussNewtonParams>::Verbosity value){ self->setVerbosity(value);}, py::arg("value"))
        .def("getVerbosity",[](gtsam::Optimizer<gtsam::GaussNewtonParams>* self){return self->getVerbosity();})
        .def("getVerbosity",[](gtsam::Optimizer<gtsam::GaussNewtonParams>* self){return self->getVerbosity();});

    py::enum_<gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity>(optimizergaussnewtonparams, "Verbosity", py::arithmetic())
        .value("SILENT", gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity::SILENT)
        .value("SUMMARY", gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity::SUMMARY)
        .value("VERBOSE", gtsam::Optimizer<gtsam::GaussNewtonParams>::Verbosity::VERBOSE);



#include "python/specializations.h"

}

