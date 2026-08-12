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

PYBIND11_MODULE(functions_py, m_) {
    m_.doc() = "pybind11 wrapper of functions_py";


    m_.def("load2D",[](string filename, std::shared_ptr<Test> model, int maxID, bool addNoise, bool smart){return ::load2D(filename, model, maxID, addNoise, smart);}, gtwrap::internal::py_arg<string>("filename"), gtwrap::internal::py_arg<std::shared_ptr<Test>>("model"), gtwrap::internal::py_arg<int>("maxID"), gtwrap::internal::py_arg<bool>("addNoise"), gtwrap::internal::py_arg<bool>("smart"));
    m_.def("load2D",[](string filename, const std::shared_ptr<gtsam::noiseModel::Diagonal> model, int maxID, bool addNoise, bool smart){return ::load2D(filename, model, maxID, addNoise, smart);}, gtwrap::internal::py_arg<string>("filename"), gtwrap::internal::py_arg<const std::shared_ptr<gtsam::noiseModel::Diagonal>>("model"), gtwrap::internal::py_arg<int>("maxID"), gtwrap::internal::py_arg<bool>("addNoise"), gtwrap::internal::py_arg<bool>("smart"));
    m_.def("load2D",[](string filename, gtsam::noiseModel::Diagonal* model){return ::load2D(filename, model);}, gtwrap::internal::py_arg<string>("filename"), gtwrap::internal::py_arg<gtsam::noiseModel::Diagonal*>("model"));
    m_.def("aGlobalFunction",[](){return ::aGlobalFunction();});
    m_.def("overloadedGlobalFunction",[](int a){return ::overloadedGlobalFunction(a);}, gtwrap::internal::py_arg<int>("a"));
    m_.def("overloadedGlobalFunction",[](int a, double b){return ::overloadedGlobalFunction(a, b);}, gtwrap::internal::py_arg<int>("a"), gtwrap::internal::py_arg<double>("b"));
    m_.def("MultiTemplatedFunctionStringSize_tDouble",[](const string& x, size_t y){return ::MultiTemplatedFunction<string,size_t,double>(x, y);}, gtwrap::internal::py_arg<const string&>("x"), gtwrap::internal::py_arg<size_t>("y"));
    m_.def("MultiTemplatedFunctionDoubleSize_tDouble",[](const double& x, size_t y){return ::MultiTemplatedFunction<double,size_t,double>(x, y);}, gtwrap::internal::py_arg<const double&>("x"), gtwrap::internal::py_arg<size_t>("y"));
    m_.def("DefaultFuncInt",[](int a, int b){ ::DefaultFuncInt(a, b);}, gtwrap::internal::py_arg<int>("a") = 123, gtwrap::internal::py_arg<int>("b") = 0);
    m_.def("DefaultFuncString",[](const string& s, const string& name){ ::DefaultFuncString(s, name);}, gtwrap::internal::py_arg<const string&>("s") = "hello", gtwrap::internal::py_arg<const string&>("name") = "");
    m_.def("DefaultFuncObj",[](const gtsam::KeyFormatter& keyFormatter){ ::DefaultFuncObj(keyFormatter);}, gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter);
    m_.def("DefaultFuncZero",[](int a, int b, double c, int d, bool e){ ::DefaultFuncZero(a, b, c, d, e);}, gtwrap::internal::py_arg<int>("a"), gtwrap::internal::py_arg<int>("b"), gtwrap::internal::py_arg<double>("c") = 0.0, gtwrap::internal::py_arg<int>("d") = 0, gtwrap::internal::py_arg<bool>("e") = false);
    m_.def("DefaultFuncVector",[](const std::vector<int>& i, const std::vector<string>& s){ ::DefaultFuncVector(i, s);}, gtwrap::internal::py_arg<const std::vector<int>&>("i") = {1, 2, 3}, gtwrap::internal::py_arg<const std::vector<string>&>("s") = {"borglab", "gtsam"});
    m_.def("setPose",[](const gtsam::Pose3& pose){ ::setPose(pose);}, gtwrap::internal::py_arg<const gtsam::Pose3&>("pose") = gtsam::Pose3());
    m_.def("EliminateDiscrete",[](const gtsam::DiscreteFactorGraph& factors, const gtsam::Ordering& frontalKeys){return ::EliminateDiscrete(factors, frontalKeys);}, gtwrap::internal::py_arg<const gtsam::DiscreteFactorGraph&>("factors"), gtwrap::internal::py_arg<const gtsam::Ordering&>("frontalKeys"));
    m_.def("triangulatePoint3Cal3_S2",[](const gtsam::Pose3Vector& poses, std::shared_ptr<gtsam::Cal3_S2> sharedCal, const gtsam::Point2Vector& measurements, double rank_tol, bool optimize, const gtsam::SharedNoiseModel& model){return ::triangulatePoint3<gtsam::Cal3_S2>(poses, sharedCal, measurements, rank_tol, optimize, model);}, gtwrap::internal::py_arg<const gtsam::Pose3Vector&>("poses"), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Cal3_S2>>("sharedCal"), gtwrap::internal::py_arg<const gtsam::Point2Vector&>("measurements"), gtwrap::internal::py_arg<double>("rank_tol"), gtwrap::internal::py_arg<bool>("optimize"), gtwrap::internal::py_arg<const gtsam::SharedNoiseModel&>("model") = nullptr);
    m_.def("FindKarcherMeanPoint3",[](const std::vector<gtsam::Point3>& elements){return ::FindKarcherMean<gtsam::Point3>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::Point3>&>("elements"));
    m_.def("FindKarcherMeanSO3",[](const std::vector<gtsam::SO3>& elements){return ::FindKarcherMean<gtsam::SO3>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::SO3>&>("elements"));
    m_.def("FindKarcherMeanSO4",[](const std::vector<gtsam::SO4>& elements){return ::FindKarcherMean<gtsam::SO4>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::SO4>&>("elements"));
    m_.def("FindKarcherMeanPose3",[](const std::vector<gtsam::Pose3>& elements){return ::FindKarcherMean<gtsam::Pose3>(elements);}, gtwrap::internal::py_arg<const std::vector<gtsam::Pose3>&>("elements"));
    m_.def("TemplatedFunctionRot3",[](const gtsam::Rot3& t){ ::TemplatedFunction<gtsam::Rot3>(t);}, gtwrap::internal::py_arg<const gtsam::Rot3&>("t"));

#include "python/specializations.h"

}

