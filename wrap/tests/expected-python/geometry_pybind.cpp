

#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "gtsam/geometry/Point2.h"
#include "gtsam/geometry/Point3.h"
#include "folder/path/to/Test.h"

#include "wrap/serialization.h"
#include <boost/serialization/export.hpp>

BOOST_CLASS_EXPORT(gtsam::Point2)
BOOST_CLASS_EXPORT(gtsam::Point3)




using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(geometry_py, m_) {
    m_.doc() = "pybind11 wrapper of geometry_py";

    pybind11::module m_gtsam = m_.def_submodule("gtsam", "gtsam submodule");

    py::class_<gtsam::Point2, std::shared_ptr<gtsam::Point2>>(m_gtsam, "Point2")
        .def(py::init<>())
        .def(py::init< double,  double>(), py::arg("x"), py::arg("y"))
        .def("x",[](gtsam::Point2* self){return self->x();})
        .def("y",[](gtsam::Point2* self){return self->y();})
        .def("dim",[](gtsam::Point2* self){return self->dim();})
        .def("returnChar",[](gtsam::Point2* self){return self->returnChar();})
        .def("argChar",[](gtsam::Point2* self, char a){ self->argChar(a);}, py::arg("a"))
        .def("argUChar",[](gtsam::Point2* self, unsigned char a){ self->argUChar(a);}, py::arg("a"))
        .def("eigenArguments",[](gtsam::Point2* self,const gtsam::Vector& v,const gtsam::Matrix& m){ self->eigenArguments(v, m);}, py::arg("v"), py::arg("m"))
        .def("vectorConfusion",[](gtsam::Point2* self){return self->vectorConfusion();})
.def("serialize",
    [](gtsam::Point2* self){
        return gtsam::serialize(*self);
    }
)
.def("deserialize",
    [](gtsam::Point2* self, string serialized){
        gtsam::deserialize(serialized, *self);
    }, py::arg("serialized"))
;

    py::class_<gtsam::Point3, std::shared_ptr<gtsam::Point3>>(m_gtsam, "Point3")
        .def(py::init< double,  double,  double>(), py::arg("x"), py::arg("y"), py::arg("z"))
        .def("norm",[](gtsam::Point3* self){return self->norm();})
.def("serialize",
    [](gtsam::Point3* self){
        return gtsam::serialize(*self);
    }
)
.def("deserialize",
    [](gtsam::Point3* self, string serialized){
        gtsam::deserialize(serialized, *self);
    }, py::arg("serialized"))

        .def_static("staticFunction",[](){return gtsam::Point3::staticFunction();})
        .def_static("StaticFunctionRet",[]( double z){return gtsam::Point3::StaticFunctionRet(z);}, py::arg("z"));

    py::class_<Test, std::shared_ptr<Test>>(m_, "Test")
        .def(py::init<>())
        .def(py::init< double, const gtsam::Matrix&>(), py::arg("a"), py::arg("b"))
        .def("return_pair",[](Test* self,const gtsam::Vector& v,const gtsam::Matrix& A){return self->return_pair(v, A);}, py::arg("v"), py::arg("A"))
        .def("return_pair",[](Test* self,const gtsam::Vector& v){return self->return_pair(v);}, py::arg("v"))
        .def("return_bool",[](Test* self, bool value){return self->return_bool(value);}, py::arg("value"))
        .def("return_size_t",[](Test* self, size_t value){return self->return_size_t(value);}, py::arg("value"))
        .def("return_int",[](Test* self, int value){return self->return_int(value);}, py::arg("value"))
        .def("return_double",[](Test* self, double value){return self->return_double(value);}, py::arg("value"))
        .def("return_string",[](Test* self, string value){return self->return_string(value);}, py::arg("value"))
        .def("return_vector1",[](Test* self,const gtsam::Vector& value){return self->return_vector1(value);}, py::arg("value"))
        .def("return_matrix1",[](Test* self,const gtsam::Matrix& value){return self->return_matrix1(value);}, py::arg("value"))
        .def("return_vector2",[](Test* self,const gtsam::Vector& value){return self->return_vector2(value);}, py::arg("value"))
        .def("return_matrix2",[](Test* self,const gtsam::Matrix& value){return self->return_matrix2(value);}, py::arg("value"))
        .def("arg_EigenConstRef",[](Test* self,const gtsam::Matrix& value){ self->arg_EigenConstRef(value);}, py::arg("value"))
        .def("return_field",[](Test* self,const Test& t){return self->return_field(t);}, py::arg("t"))
        .def("return_TestPtr",[](Test* self,const std::shared_ptr<Test>& value){return self->return_TestPtr(value);}, py::arg("value"))
        .def("return_Test",[](Test* self,const std::shared_ptr<Test>& value){return self->return_Test(value);}, py::arg("value"))
        .def("return_Point2Ptr",[](Test* self, bool value){return self->return_Point2Ptr(value);}, py::arg("value"))
        .def("create_ptrs",[](Test* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](Test* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](Test* self,const std::shared_ptr<Test>& p1,const std::shared_ptr<Test>& p2){return self->return_ptrs(p1, p2);}, py::arg("p1"), py::arg("p2"))
        .def("print_",[](Test* self){ self->print();})
        .def("__repr__",
                    [](const Test &a) {
                        gtsam::RedirectCout redirect;
                        a.print();
                        return redirect.str();
                    });

    py::class_<MyBase, std::shared_ptr<MyBase>>(m_, "MyBase");

    py::class_<MyTemplate<gtsam::Point2>, MyBase, std::shared_ptr<MyTemplate<gtsam::Point2>>>(m_, "MyTemplatePoint2")
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<gtsam::Point2>* self,const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, py::arg("t"))
        .def("templatedMethodPoint3",[](MyTemplate<gtsam::Point2>* self,const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, py::arg("t"))
        .def("templatedMethodVector",[](MyTemplate<gtsam::Point2>* self,const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, py::arg("t"))
        .def("templatedMethodMatrix",[](MyTemplate<gtsam::Point2>* self,const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, py::arg("t"))
        .def("accept_T",[](MyTemplate<gtsam::Point2>* self,const gtsam::Point2& value){ self->accept_T(value);}, py::arg("value"))
        .def("accept_Tptr",[](MyTemplate<gtsam::Point2>* self,const std::shared_ptr<gtsam::Point2>& value){ self->accept_Tptr(value);}, py::arg("value"))
        .def("return_Tptr",[](MyTemplate<gtsam::Point2>* self,const std::shared_ptr<gtsam::Point2>& value){return self->return_Tptr(value);}, py::arg("value"))
        .def("return_T",[](MyTemplate<gtsam::Point2>* self,const std::shared_ptr<gtsam::Point2>& value){return self->return_T(value);}, py::arg("value"))
        .def("create_ptrs",[](MyTemplate<gtsam::Point2>* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](MyTemplate<gtsam::Point2>* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](MyTemplate<gtsam::Point2>* self,const std::shared_ptr<gtsam::Point2>& p1,const std::shared_ptr<gtsam::Point2>& p2){return self->return_ptrs(p1, p2);}, py::arg("p1"), py::arg("p2"))
        .def_static("Level",[](const gtsam::Point2& K){return MyTemplate<gtsam::Point2>::Level(K);}, py::arg("K"));

    py::class_<MyTemplate<gtsam::Matrix>, MyBase, std::shared_ptr<MyTemplate<gtsam::Matrix>>>(m_, "MyTemplateMatrix")
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<gtsam::Matrix>* self,const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, py::arg("t"))
        .def("templatedMethodPoint3",[](MyTemplate<gtsam::Matrix>* self,const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, py::arg("t"))
        .def("templatedMethodVector",[](MyTemplate<gtsam::Matrix>* self,const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, py::arg("t"))
        .def("templatedMethodMatrix",[](MyTemplate<gtsam::Matrix>* self,const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, py::arg("t"))
        .def("accept_T",[](MyTemplate<gtsam::Matrix>* self,const gtsam::Matrix& value){ self->accept_T(value);}, py::arg("value"))
        .def("accept_Tptr",[](MyTemplate<gtsam::Matrix>* self,const std::shared_ptr<gtsam::Matrix>& value){ self->accept_Tptr(value);}, py::arg("value"))
        .def("return_Tptr",[](MyTemplate<gtsam::Matrix>* self,const std::shared_ptr<gtsam::Matrix>& value){return self->return_Tptr(value);}, py::arg("value"))
        .def("return_T",[](MyTemplate<gtsam::Matrix>* self,const std::shared_ptr<gtsam::Matrix>& value){return self->return_T(value);}, py::arg("value"))
        .def("create_ptrs",[](MyTemplate<gtsam::Matrix>* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](MyTemplate<gtsam::Matrix>* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](MyTemplate<gtsam::Matrix>* self,const std::shared_ptr<gtsam::Matrix>& p1,const std::shared_ptr<gtsam::Matrix>& p2){return self->return_ptrs(p1, p2);}, py::arg("p1"), py::arg("p2"))
        .def_static("Level",[](const gtsam::Matrix& K){return MyTemplate<gtsam::Matrix>::Level(K);}, py::arg("K"));

    py::class_<PrimitiveRef<double>, std::shared_ptr<PrimitiveRef<double>>>(m_, "PrimitiveRefdouble")
        .def(py::init<>())
        .def_static("Brutal",[](const double& t){return PrimitiveRef<double>::Brutal(t);}, py::arg("t"));

    py::class_<MyVector<3>, std::shared_ptr<MyVector<3>>>(m_, "MyVector3")
        .def(py::init<>());

    py::class_<MyVector<12>, std::shared_ptr<MyVector<12>>>(m_, "MyVector12")
        .def(py::init<>());

    py::class_<MyFactor<gtsam::Pose2, gtsam::Matrix>, std::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>>>(m_, "MyFactorPosePoint2")
        .def(py::init< size_t,  size_t,  double, const std::shared_ptr<gtsam::noiseModel::Base>&>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"));

    m_.def("load2D",[]( string filename,const std::shared_ptr<Test>& model, int maxID, bool addNoise, bool smart){return ::load2D(filename, model, maxID, addNoise, smart);}, py::arg("filename"), py::arg("model"), py::arg("maxID"), py::arg("addNoise"), py::arg("smart"));
    m_.def("load2D",[]( string filename,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model, int maxID, bool addNoise, bool smart){return ::load2D(filename, model, maxID, addNoise, smart);}, py::arg("filename"), py::arg("model"), py::arg("maxID"), py::arg("addNoise"), py::arg("smart"));
    m_.def("load2D",[]( string filename,const std::shared_ptr<gtsam::noiseModel::Diagonal>& model){return ::load2D(filename, model);}, py::arg("filename"), py::arg("model"));
    m_.def("aGlobalFunction",[](){return ::aGlobalFunction();});
    m_.def("overloadedGlobalFunction",[]( int a){return ::overloadedGlobalFunction(a);}, py::arg("a"));
    m_.def("overloadedGlobalFunction",[]( int a, double b){return ::overloadedGlobalFunction(a, b);}, py::arg("a"), py::arg("b"));

#include "python/specializations.h"

}

