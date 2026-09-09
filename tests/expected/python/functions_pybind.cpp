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



void gtwrap_declare_functions_py(py::module_ &m_) {

}

void gtwrap_bind_functions_py(py::module_ &m_) {
#include "python/specializations.h"

    m_.def("load2D",static_cast<std::pair<std::shared_ptr<gtsam::NonlinearFactorGraph>,std::shared_ptr<gtsam::Values>> (*)(string, std::shared_ptr<Test>, int, bool, bool)>(&::load2D), gtwrap::internal::py_arg<string>("filename"), gtwrap::internal::py_arg<std::shared_ptr<Test>>("model"), gtwrap::internal::py_arg<int>("maxID"), gtwrap::internal::py_arg<bool>("addNoise"), gtwrap::internal::py_arg<bool>("smart"));
    m_.def("load2D",static_cast<std::pair<std::shared_ptr<gtsam::NonlinearFactorGraph>,std::shared_ptr<gtsam::Values>> (*)(string, const std::shared_ptr<gtsam::noiseModel::Diagonal>, int, bool, bool)>(&::load2D), gtwrap::internal::py_arg<string>("filename"), gtwrap::internal::py_arg<const std::shared_ptr<gtsam::noiseModel::Diagonal>>("model"), gtwrap::internal::py_arg<int>("maxID"), gtwrap::internal::py_arg<bool>("addNoise"), gtwrap::internal::py_arg<bool>("smart"));
    m_.def("load2D",static_cast<std::pair<std::shared_ptr<gtsam::NonlinearFactorGraph>,std::shared_ptr<gtsam::Values>> (*)(string, gtsam::noiseModel::Diagonal*)>(&::load2D), gtwrap::internal::py_arg<string>("filename"), gtwrap::internal::py_arg<gtsam::noiseModel::Diagonal*>("model"));
    m_.def("aGlobalFunction",static_cast<gtsam::Vector (*)()>(&::aGlobalFunction));
    m_.def("overloadedGlobalFunction",static_cast<gtsam::Vector (*)(int)>(&::overloadedGlobalFunction), gtwrap::internal::py_arg<int>("a"));
    m_.def("overloadedGlobalFunction",static_cast<gtsam::Vector (*)(int, double)>(&::overloadedGlobalFunction), gtwrap::internal::py_arg<int>("a"), gtwrap::internal::py_arg<double>("b"));
    m_.def("MultiTemplatedFunctionStringSize_tDouble",[](const string& x, size_t y){return ::MultiTemplatedFunction<string,size_t,double>(x, y);}, gtwrap::internal::py_arg<const string&>("x"), gtwrap::internal::py_arg<size_t>("y"));
    m_.def("MultiTemplatedFunctionDoubleSize_tDouble",[](const double& x, size_t y){return ::MultiTemplatedFunction<double,size_t,double>(x, y);}, gtwrap::internal::py_arg<const double&>("x"), gtwrap::internal::py_arg<size_t>("y"));
    m_.def("DefaultFuncInt",static_cast<void (*)(int, int)>(&::DefaultFuncInt), gtwrap::internal::py_arg<int>("a") = 123, gtwrap::internal::py_arg<int>("b") = 0);
    m_.def("DefaultFuncString",static_cast<void (*)(const string&, const string&)>(&::DefaultFuncString), gtwrap::internal::py_arg<const string&>("s") = "hello", gtwrap::internal::py_arg<const string&>("name") = "");
    m_.def("DefaultFuncObj",static_cast<void (*)(const gtsam::KeyFormatter&)>(&::DefaultFuncObj), gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter);
    m_.def("DefaultFuncZero",static_cast<void (*)(int, int, double, int, bool)>(&::DefaultFuncZero), gtwrap::internal::py_arg<int>("a"), gtwrap::internal::py_arg<int>("b"), gtwrap::internal::py_arg<double>("c") = 0.0, gtwrap::internal::py_arg<int>("d") = 0, gtwrap::internal::py_arg<bool>("e") = false);
    m_.def("DefaultFuncVector",static_cast<void (*)(const std::vector<int>&, const std::vector<string>&)>(&::DefaultFuncVector), gtwrap::internal::py_arg<const std::vector<int>&>("i") = {1, 2, 3}, gtwrap::internal::py_arg<const std::vector<string>&>("s") = {"borglab", "gtsam"});
    m_.def("setPose",static_cast<void (*)(const gtsam::Pose3&)>(&::setPose), gtwrap::internal::py_arg<const gtsam::Pose3&>("pose") = gtsam::Pose3());
    m_.def("EliminateDiscrete",static_cast<std::pair<std::shared_ptr<gtsam::DiscreteConditional>,std::shared_ptr<gtsam::DecisionTreeFactor>> (*)(const gtsam::DiscreteFactorGraph&, const gtsam::Ordering&)>(&::EliminateDiscrete), gtwrap::internal::py_arg<const gtsam::DiscreteFactorGraph&>("factors"), gtwrap::internal::py_arg<const gtsam::Ordering&>("frontalKeys"));
    m_.def("triangulatePoint3Cal3_S2",[](const gtsam::Pose3Vector& poses, std::shared_ptr<gtsam::Cal3_S2> sharedCal, const gtsam::Point2Vector& measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel& model){return ::triangulatePoint3<gtsam::Cal3_S2>(poses, sharedCal, measurements, rank_tol, optimize, model);}, gtwrap::internal::py_arg<const gtsam::Pose3Vector&>("poses"), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Cal3_S2>>("sharedCal"), gtwrap::internal::py_arg<const gtsam::Point2Vector&>("measurements"), gtwrap::internal::py_arg<double>("rank_tol"), gtwrap::internal::py_arg<bool>("optimize"), gtwrap::internal::py_arg<const gtsam::SharedNoiseModel&>("model") = nullptr);
    m_.def("FindKarcherMeanPoint3",[](const std::vector<gtsam::Point3>& elements){return ::FindKarcherMean<gtsam::Point3>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::Point3>&>("elements"));
    m_.def("FindKarcherMeanSO3",[](const std::vector<gtsam::SO3>& elements){return ::FindKarcherMean<gtsam::SO3>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::SO3>&>("elements"));
    m_.def("FindKarcherMeanSO4",[](const std::vector<gtsam::SO4>& elements){return ::FindKarcherMean<gtsam::SO4>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::SO4>&>("elements"));
    m_.def("FindKarcherMeanPose3",[](const std::vector<gtsam::Pose3>& elements){return ::FindKarcherMean<gtsam::Pose3>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::Pose3>&>("elements"));
    m_.def("TemplatedFunctionRot3",[](const gtsam::Rot3& t){ ::TemplatedFunction<gtsam::Rot3>(t);}, gtwrap::internal::py_arg<const gtsam::Rot3&>("t"));
}

PYBIND11_MODULE(functions_py, m_) {
    m_.doc() = "pybind11 wrapper of functions_py";

gtwrap_declare_functions_py(m_);
gtwrap_bind_functions_py(m_);
}
