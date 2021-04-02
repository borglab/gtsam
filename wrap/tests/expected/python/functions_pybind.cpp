

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
    m_.def("TemplatedFunctionRot3",[](const gtsam::Rot3& t){ ::TemplatedFunction<Rot3>(t);}, py::arg("t"));

#include "python/specializations.h"

}

