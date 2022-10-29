

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.


#include "wrap/serialization.h"
#include <boost/serialization/export.hpp>





using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(functions_py, m_) {
    m_.doc() = "pybind11 wrapper of functions_py";


    m_.def("load2D",[](string filename, std::shared_ptr<Test> model, int maxID, bool addNoise, bool smart){return ::load2D(filename, model, maxID, addNoise, smart);}, py::arg("filename"), py::arg("model"), py::arg("maxID"), py::arg("addNoise"), py::arg("smart"));
    m_.def("load2D",[](string filename, const std::shared_ptr<gtsam::noiseModel::Diagonal> model, int maxID, bool addNoise, bool smart){return ::load2D(filename, model, maxID, addNoise, smart);}, py::arg("filename"), py::arg("model"), py::arg("maxID"), py::arg("addNoise"), py::arg("smart"));
    m_.def("load2D",[](string filename, gtsam::noiseModel::Diagonal* model){return ::load2D(filename, model);}, py::arg("filename"), py::arg("model"));
    m_.def("aGlobalFunction",[](){return ::aGlobalFunction();});
    m_.def("overloadedGlobalFunction",[](int a){return ::overloadedGlobalFunction(a);}, py::arg("a"));
    m_.def("overloadedGlobalFunction",[](int a, double b){return ::overloadedGlobalFunction(a, b);}, py::arg("a"), py::arg("b"));
    m_.def("MultiTemplatedFunctionStringSize_tDouble",[](const T& x, size_t y){return ::MultiTemplatedFunction<string,size_t,double>(x, y);}, py::arg("x"), py::arg("y"));
    m_.def("MultiTemplatedFunctionDoubleSize_tDouble",[](const T& x, size_t y){return ::MultiTemplatedFunction<double,size_t,double>(x, y);}, py::arg("x"), py::arg("y"));
    m_.def("DefaultFuncInt",[](int a, int b){ ::DefaultFuncInt(a, b);}, py::arg("a") = 123, py::arg("b") = 0);
    m_.def("DefaultFuncString",[](const string& s, const string& name){ ::DefaultFuncString(s, name);}, py::arg("s") = "hello", py::arg("name") = "");
    m_.def("DefaultFuncObj",[](const gtsam::KeyFormatter& keyFormatter){ ::DefaultFuncObj(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);
    m_.def("DefaultFuncZero",[](int a, int b, double c, int d, bool e){ ::DefaultFuncZero(a, b, c, d, e);}, py::arg("a"), py::arg("b"), py::arg("c") = 0.0, py::arg("d") = 0, py::arg("e") = false);
    m_.def("DefaultFuncVector",[](const std::vector<int>& i, const std::vector<string>& s){ ::DefaultFuncVector(i, s);}, py::arg("i") = {1, 2, 3}, py::arg("s") = {"borglab", "gtsam"});
    m_.def("setPose",[](const gtsam::Pose3& pose){ ::setPose(pose);}, py::arg("pose") = gtsam::Pose3());
    m_.def("TemplatedFunctionRot3",[](const gtsam::Rot3& t){ ::TemplatedFunction<gtsam::Rot3>(t);}, py::arg("t"));

#include "python/specializations.h"

}

