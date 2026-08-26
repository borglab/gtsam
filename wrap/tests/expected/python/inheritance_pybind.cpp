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



void gtwrap_declare_inheritance_py(py::module_ &m_) {

    py::class_<MyBase, std::shared_ptr<MyBase>>(m_, "MyBase");

    py::class_<MyTemplate<gtsam::Point2>, MyBase, std::shared_ptr<MyTemplate<gtsam::Point2>>>(m_, "MyTemplatePoint2");

    py::class_<MyTemplate<gtsam::Matrix>, MyBase, std::shared_ptr<MyTemplate<gtsam::Matrix>>>(m_, "MyTemplateMatrix");

    py::class_<MyTemplate<A>, MyBase, std::shared_ptr<MyTemplate<A>>>(m_, "MyTemplateA");

    py::class_<ForwardKinematicsFactor, gtsam::BetweenFactor<gtsam::Pose3>, std::shared_ptr<ForwardKinematicsFactor>>(m_, "ForwardKinematicsFactor");

    py::class_<ParentHasTemplate<double>, MyTemplate<double>, std::shared_ptr<ParentHasTemplate<double>>>(m_, "ParentHasTemplateDouble");

    py::class_<Base, std::shared_ptr<Base>>(m_, "Base");

    py::class_<Derived, Base, std::shared_ptr<Derived>>(m_, "Derived");

}

void gtwrap_bind_inheritance_py(py::module_ &m_) {
#include "python/specializations.h"

    auto gtwrap_class_m__MyTemplatePoint2 = py::reinterpret_borrow<py::class_<MyTemplate<gtsam::Point2>, MyBase, std::shared_ptr<MyTemplate<gtsam::Point2>>>>(m_.attr("MyTemplatePoint2"));
    gtwrap_class_m__MyTemplatePoint2
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<gtsam::Point2>* self, const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, gtwrap::internal::py_arg<const gtsam::Point2&>("t"))
        .def("templatedMethodPoint3",[](MyTemplate<gtsam::Point2>* self, const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, gtwrap::internal::py_arg<const gtsam::Point3&>("t"))
        .def("templatedMethodVector",[](MyTemplate<gtsam::Point2>* self, const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, gtwrap::internal::py_arg<const gtsam::Vector&>("t"))
        .def("templatedMethodMatrix",[](MyTemplate<gtsam::Point2>* self, const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, gtwrap::internal::py_arg<const gtsam::Matrix&>("t"))
        .def("accept_T",static_cast<void (MyTemplate<gtsam::Point2>::*)(const gtsam::Point2&) const>(&MyTemplate<gtsam::Point2>::accept_T), gtwrap::internal::py_arg<const gtsam::Point2&>("value"))
        .def("accept_Tptr",static_cast<void (MyTemplate<gtsam::Point2>::*)(std::shared_ptr<gtsam::Point2>) const>(&MyTemplate<gtsam::Point2>::accept_Tptr), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Point2>>("value"))
        .def("return_Tptr",static_cast<std::shared_ptr<gtsam::Point2> (MyTemplate<gtsam::Point2>::*)(std::shared_ptr<gtsam::Point2>) const>(&MyTemplate<gtsam::Point2>::return_Tptr), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Point2>>("value"))
        .def("return_T",static_cast<gtsam::Point2 (MyTemplate<gtsam::Point2>::*)(gtsam::Point2*) const>(&MyTemplate<gtsam::Point2>::return_T), gtwrap::internal::py_arg<gtsam::Point2*>("value"))
        .def("create_ptrs",static_cast<std::pair<std::shared_ptr<gtsam::Point2>,std::shared_ptr<gtsam::Point2>> (MyTemplate<gtsam::Point2>::*)() const>(&MyTemplate<gtsam::Point2>::create_ptrs))
        .def("create_MixedPtrs",static_cast<std::pair<gtsam::Point2,std::shared_ptr<gtsam::Point2>> (MyTemplate<gtsam::Point2>::*)() const>(&MyTemplate<gtsam::Point2>::create_MixedPtrs))
        .def("return_ptrs",static_cast<std::pair<std::shared_ptr<gtsam::Point2>,std::shared_ptr<gtsam::Point2>> (MyTemplate<gtsam::Point2>::*)(std::shared_ptr<gtsam::Point2>, std::shared_ptr<gtsam::Point2>) const>(&MyTemplate<gtsam::Point2>::return_ptrs), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Point2>>("p1"), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Point2>>("p2"))
        .def_static("Level",static_cast<MyTemplate<gtsam::Point2> (*)(const gtsam::Point2&)>(&MyTemplate<gtsam::Point2>::Level), gtwrap::internal::py_arg<const gtsam::Point2&>("K"));

    auto gtwrap_class_m__MyTemplateMatrix = py::reinterpret_borrow<py::class_<MyTemplate<gtsam::Matrix>, MyBase, std::shared_ptr<MyTemplate<gtsam::Matrix>>>>(m_.attr("MyTemplateMatrix"));
    gtwrap_class_m__MyTemplateMatrix
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, gtwrap::internal::py_arg<const gtsam::Point2&>("t"))
        .def("templatedMethodPoint3",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, gtwrap::internal::py_arg<const gtsam::Point3&>("t"))
        .def("templatedMethodVector",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, gtwrap::internal::py_arg<const gtsam::Vector&>("t"))
        .def("templatedMethodMatrix",[](MyTemplate<gtsam::Matrix>* self, const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, gtwrap::internal::py_arg<const gtsam::Matrix&>("t"))
        .def("accept_T",static_cast<void (MyTemplate<gtsam::Matrix>::*)(const gtsam::Matrix&) const>(&MyTemplate<gtsam::Matrix>::accept_T), gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("accept_Tptr",static_cast<void (MyTemplate<gtsam::Matrix>::*)(std::shared_ptr<gtsam::Matrix>) const>(&MyTemplate<gtsam::Matrix>::accept_Tptr), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Matrix>>("value"))
        .def("return_Tptr",static_cast<std::shared_ptr<gtsam::Matrix> (MyTemplate<gtsam::Matrix>::*)(std::shared_ptr<gtsam::Matrix>) const>(&MyTemplate<gtsam::Matrix>::return_Tptr), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Matrix>>("value"))
        .def("return_T",static_cast<gtsam::Matrix (MyTemplate<gtsam::Matrix>::*)(gtsam::Matrix*) const>(&MyTemplate<gtsam::Matrix>::return_T), gtwrap::internal::py_arg<gtsam::Matrix*>("value"))
        .def("create_ptrs",static_cast<std::pair<std::shared_ptr<gtsam::Matrix>,std::shared_ptr<gtsam::Matrix>> (MyTemplate<gtsam::Matrix>::*)() const>(&MyTemplate<gtsam::Matrix>::create_ptrs))
        .def("create_MixedPtrs",static_cast<std::pair<gtsam::Matrix,std::shared_ptr<gtsam::Matrix>> (MyTemplate<gtsam::Matrix>::*)() const>(&MyTemplate<gtsam::Matrix>::create_MixedPtrs))
        .def("return_ptrs",static_cast<std::pair<std::shared_ptr<gtsam::Matrix>,std::shared_ptr<gtsam::Matrix>> (MyTemplate<gtsam::Matrix>::*)(std::shared_ptr<gtsam::Matrix>, std::shared_ptr<gtsam::Matrix>) const>(&MyTemplate<gtsam::Matrix>::return_ptrs), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Matrix>>("p1"), gtwrap::internal::py_arg<std::shared_ptr<gtsam::Matrix>>("p2"))
        .def_static("Level",static_cast<MyTemplate<gtsam::Matrix> (*)(const gtsam::Matrix&)>(&MyTemplate<gtsam::Matrix>::Level), gtwrap::internal::py_arg<const gtsam::Matrix&>("K"));

    auto gtwrap_class_m__MyTemplateA = py::reinterpret_borrow<py::class_<MyTemplate<A>, MyBase, std::shared_ptr<MyTemplate<A>>>>(m_.attr("MyTemplateA"));
    gtwrap_class_m__MyTemplateA
        .def(py::init<>())
        .def("templatedMethodPoint2",[](MyTemplate<A>* self, const gtsam::Point2& t){return self->templatedMethod<gtsam::Point2>(t);}, gtwrap::internal::py_arg<const gtsam::Point2&>("t"))
        .def("templatedMethodPoint3",[](MyTemplate<A>* self, const gtsam::Point3& t){return self->templatedMethod<gtsam::Point3>(t);}, gtwrap::internal::py_arg<const gtsam::Point3&>("t"))
        .def("templatedMethodVector",[](MyTemplate<A>* self, const gtsam::Vector& t){return self->templatedMethod<gtsam::Vector>(t);}, gtwrap::internal::py_arg<const gtsam::Vector&>("t"))
        .def("templatedMethodMatrix",[](MyTemplate<A>* self, const gtsam::Matrix& t){return self->templatedMethod<gtsam::Matrix>(t);}, gtwrap::internal::py_arg<const gtsam::Matrix&>("t"))
        .def("accept_T",static_cast<void (MyTemplate<A>::*)(const A&) const>(&MyTemplate<A>::accept_T), gtwrap::internal::py_arg<const A&>("value"))
        .def("accept_Tptr",static_cast<void (MyTemplate<A>::*)(std::shared_ptr<A>) const>(&MyTemplate<A>::accept_Tptr), gtwrap::internal::py_arg<std::shared_ptr<A>>("value"))
        .def("return_Tptr",static_cast<std::shared_ptr<A> (MyTemplate<A>::*)(std::shared_ptr<A>) const>(&MyTemplate<A>::return_Tptr), gtwrap::internal::py_arg<std::shared_ptr<A>>("value"))
        .def("return_T",static_cast<A (MyTemplate<A>::*)(A*) const>(&MyTemplate<A>::return_T), gtwrap::internal::py_arg<A*>("value"))
        .def("create_ptrs",static_cast<std::pair<std::shared_ptr<A>,std::shared_ptr<A>> (MyTemplate<A>::*)() const>(&MyTemplate<A>::create_ptrs))
        .def("create_MixedPtrs",static_cast<std::pair<A,std::shared_ptr<A>> (MyTemplate<A>::*)() const>(&MyTemplate<A>::create_MixedPtrs))
        .def("return_ptrs",static_cast<std::pair<std::shared_ptr<A>,std::shared_ptr<A>> (MyTemplate<A>::*)(std::shared_ptr<A>, std::shared_ptr<A>) const>(&MyTemplate<A>::return_ptrs), gtwrap::internal::py_arg<std::shared_ptr<A>>("p1"), gtwrap::internal::py_arg<std::shared_ptr<A>>("p2"))
        .def_static("Level",static_cast<MyTemplate<A> (*)(const A&)>(&MyTemplate<A>::Level), gtwrap::internal::py_arg<const A&>("K"));

    auto gtwrap_class_m__Base = py::reinterpret_borrow<py::class_<Base, std::shared_ptr<Base>>>(m_.attr("Base"));
    gtwrap_class_m__Base
        .def_static("Create",static_cast<std::shared_ptr<gtsam::Base> (*)(double)>(&Base::Create), gtwrap::internal::py_arg<double>("x"));

}

PYBIND11_MODULE(inheritance_py, m_) {
    m_.doc() = "pybind11 wrapper of inheritance_py";

gtwrap_declare_inheritance_py(m_);
gtwrap_bind_inheritance_py(m_);
}
