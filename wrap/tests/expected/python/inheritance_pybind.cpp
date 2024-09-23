#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.





using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(inheritance_py, m_) {
    m_.doc() = "pybind11 wrapper of inheritance_py";


    py::class_<MyBase, std::shared_ptr<MyBase>>(m_, "MyBase");

    py::class_<MyTemplate<gtsam::Point2>, MyBase, std::shared_ptr<MyTemplate<gtsam::Point2>>>(m_, "MyTemplatePoint2")
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<gtsam::Point2>* self, const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, py::arg("t"))
        .def("templatedMethodPoint3",[](MyTemplate<gtsam::Point2>* self, const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, py::arg("t"))
        .def("templatedMethodVector",[](MyTemplate<gtsam::Point2>* self, const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, py::arg("t"))
        .def("templatedMethodMatrix",[](MyTemplate<gtsam::Point2>* self, const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, py::arg("t"))
        .def("accept_T",[](MyTemplate<gtsam::Point2>* self, const gtsam::Point2& value){ self->accept_T(value);}, py::arg("value"))
        .def("accept_Tptr",[](MyTemplate<gtsam::Point2>* self, std::shared_ptr<gtsam::Point2> value){ self->accept_Tptr(value);}, py::arg("value"))
        .def("return_Tptr",[](MyTemplate<gtsam::Point2>* self, std::shared_ptr<gtsam::Point2> value){return self->return_Tptr(value);}, py::arg("value"))
        .def("return_T",[](MyTemplate<gtsam::Point2>* self, gtsam::Point2* value){return self->return_T(value);}, py::arg("value"))
        .def("create_ptrs",[](MyTemplate<gtsam::Point2>* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](MyTemplate<gtsam::Point2>* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](MyTemplate<gtsam::Point2>* self, std::shared_ptr<gtsam::Point2> p1, std::shared_ptr<gtsam::Point2> p2){return self->return_ptrs(p1, p2);}, py::arg("p1"), py::arg("p2"))
        .def_static("Level",[](const gtsam::Point2& K){return MyTemplate<gtsam::Point2>::Level(K);}, py::arg("K"));

    py::class_<MyTemplate<gtsam::Matrix>, MyBase, std::shared_ptr<MyTemplate<gtsam::Matrix>>>(m_, "MyTemplateMatrix")
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, py::arg("t"))
        .def("templatedMethodPoint3",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, py::arg("t"))
        .def("templatedMethodVector",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, py::arg("t"))
        .def("templatedMethodMatrix",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, py::arg("t"))
        .def("accept_T",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Matrix& value){ self->accept_T(value);}, py::arg("value"))
        .def("accept_Tptr",[](MyTemplate<gtsam::Matrix>* self, std::shared_ptr<gtsam::Matrix> value){ self->accept_Tptr(value);}, py::arg("value"))
        .def("return_Tptr",[](MyTemplate<gtsam::Matrix>* self, std::shared_ptr<gtsam::Matrix> value){return self->return_Tptr(value);}, py::arg("value"))
        .def("return_T",[](MyTemplate<gtsam::Matrix>* self, gtsam::Matrix* value){return self->return_T(value);}, py::arg("value"))
        .def("create_ptrs",[](MyTemplate<gtsam::Matrix>* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](MyTemplate<gtsam::Matrix>* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](MyTemplate<gtsam::Matrix>* self, std::shared_ptr<gtsam::Matrix> p1, std::shared_ptr<gtsam::Matrix> p2){return self->return_ptrs(p1, p2);}, py::arg("p1"), py::arg("p2"))
        .def_static("Level",[](const gtsam::Matrix& K){return MyTemplate<gtsam::Matrix>::Level(K);}, py::arg("K"));

    py::class_<MyTemplate<A>, MyBase, std::shared_ptr<MyTemplate<A>>>(m_, "MyTemplateA")
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<A>* self, const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, py::arg("t"))
        .def("templatedMethodPoint3",[](MyTemplate<A>* self, const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, py::arg("t"))
        .def("templatedMethodVector",[](MyTemplate<A>* self, const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, py::arg("t"))
        .def("templatedMethodMatrix",[](MyTemplate<A>* self, const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, py::arg("t"))
        .def("accept_T",[](MyTemplate<A>* self, const A& value){ self->accept_T(value);}, py::arg("value"))
        .def("accept_Tptr",[](MyTemplate<A>* self, std::shared_ptr<A> value){ self->accept_Tptr(value);}, py::arg("value"))
        .def("return_Tptr",[](MyTemplate<A>* self, std::shared_ptr<A> value){return self->return_Tptr(value);}, py::arg("value"))
        .def("return_T",[](MyTemplate<A>* self, A* value){return self->return_T(value);}, py::arg("value"))
        .def("create_ptrs",[](MyTemplate<A>* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](MyTemplate<A>* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](MyTemplate<A>* self, std::shared_ptr<A> p1, std::shared_ptr<A> p2){return self->return_ptrs(p1, p2);}, py::arg("p1"), py::arg("p2"))
        .def_static("Level",[](const A& K){return MyTemplate<A>::Level(K);}, py::arg("K"));

    py::class_<ForwardKinematicsFactor, gtsam::BetweenFactor<gtsam::Pose3>, std::shared_ptr<ForwardKinematicsFactor>>(m_, "ForwardKinematicsFactor");

    py::class_<ParentHasTemplate<double>, MyTemplate<double>, std::shared_ptr<ParentHasTemplate<double>>>(m_, "ParentHasTemplateDouble");


#include "python/specializations.h"

}

