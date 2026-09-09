#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "gtsam/geometry/Cal3Bundler.h"

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



void gtwrap_declare_special_cases_py(py::module_ &m_) {

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::NonlinearFactorGraph, std::shared_ptr<gtsam::NonlinearFactorGraph>>(m_gtsam, "NonlinearFactorGraph");

    py::class_<gtsam::SfmTrack, std::shared_ptr<gtsam::SfmTrack>>(m_gtsam, "SfmTrack");

    py::class_<gtsam::PinholeCamera<gtsam::Cal3Bundler>, std::shared_ptr<gtsam::PinholeCamera<gtsam::Cal3Bundler>>>(m_gtsam, "PinholeCameraCal3Bundler");

    py::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>, std::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>>> gtwrap_class_m_gtsam_GeneralSFMFactorCal3Bundler(m_gtsam, "GeneralSFMFactorCal3Bundler");
    py::enum_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity>(gtwrap_class_m_gtsam_GeneralSFMFactorCal3Bundler, "Verbosity", py::arithmetic())
        .value("SILENT", gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity::SILENT)
        .value("SUMMARY", gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity::SUMMARY)
        .value("VALUES", gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::Verbosity::VALUES);


}

void gtwrap_bind_special_cases_py(py::module_ &m_) {
#include "python/specializations.h"

    pybind11::module m_gtsam = py::reinterpret_borrow<pybind11::module>(m_.attr("gtsam"));

    auto gtwrap_class_m_gtsam_NonlinearFactorGraph = py::reinterpret_borrow<py::class_<gtsam::NonlinearFactorGraph, std::shared_ptr<gtsam::NonlinearFactorGraph>>>(m_gtsam.attr("NonlinearFactorGraph"));
    gtwrap_class_m_gtsam_NonlinearFactorGraph
        .def("addPriorPinholeCameraCal3Bundler",[](gtsam::NonlinearFactorGraph* self, size_t key, const gtsam::PinholeCamera<gtsam::Cal3Bundler>& prior, const std::shared_ptr<gtsam::noiseModel::Base> noiseModel){ self->addPrior<gtsam::PinholeCamera<gtsam::Cal3Bundler>>(key, prior, noiseModel);}, gtwrap::internal::py_arg<size_t>("key"), gtwrap::internal::py_arg<const gtsam::PinholeCamera<gtsam::Cal3Bundler>&>("prior"), gtwrap::internal::py_arg<const std::shared_ptr<gtsam::noiseModel::Base>>("noiseModel"));

    auto gtwrap_class_m_gtsam_SfmTrack = py::reinterpret_borrow<py::class_<gtsam::SfmTrack, std::shared_ptr<gtsam::SfmTrack>>>(m_gtsam.attr("SfmTrack"));
    gtwrap_class_m_gtsam_SfmTrack
        .def_readwrite("measurements", &gtsam::SfmTrack::measurements);

    auto gtwrap_class_m_gtsam_GeneralSFMFactorCal3Bundler = py::reinterpret_borrow<py::class_<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>, std::shared_ptr<gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>>>>(m_gtsam.attr("GeneralSFMFactorCal3Bundler"));
    gtwrap_class_m_gtsam_GeneralSFMFactorCal3Bundler
        .def_readwrite("verbosity", &gtsam::GeneralSFMFactor<gtsam::PinholeCamera<gtsam::Cal3Bundler>, gtsam::Point3>::verbosity);

}

PYBIND11_MODULE(special_cases_py, m_) {
    m_.doc() = "pybind11 wrapper of special_cases_py";

gtwrap_declare_special_cases_py(m_);
gtwrap_bind_special_cases_py(m_);
}
