#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "gtsam/geometry/Cal3Bundler.h"




using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(special_cases_py, m_) {
    m_.doc() = "pybind11 wrapper of special_cases_py";

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::NonlinearFactorGraph, std::shared_ptr<gtsam::NonlinearFactorGraph>>(m_gtsam, "NonlinearFactorGraph")
        .def("addPriorPinholeCameraCal3Bundler",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& prior, const std::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key, prior, noiseModel);}, py::arg("key"), py::arg("prior"), py::arg("noiseModel"));

    py::class_<gtsam::SfmTrack, std::shared_ptr<gtsam::SfmTrack>>(m_gtsam, "SfmTrack")
        .def_readwrite("measurements", &gtsam::SfmTrack::measurements);

    py::class_<gtsam::PinholeCamera<gtsam::Cal3Bundler>, std::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>(m_gtsam, "PinholeCameraCal3Bundler");

    py::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>, std::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>>> generalsfmfactorcal3bundler(m_gtsam, "GeneralSFMFactorCal3Bundler");
    generalsfmfactorcal3bundler
        .def_readwrite("verbosity", &gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::verbosity);

    py::enum_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity>(generalsfmfactorcal3bundler, "Verbosity", py::arithmetic())
        .value("SILENT", gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity::SILENT)
        .value("SUMMARY", gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity::SUMMARY)
        .value("VALUES", gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity::VALUES);



#include "python/specializations.h"

}

