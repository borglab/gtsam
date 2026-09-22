#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "folder/path/to/Test.h"

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



void gtwrap_declare_class_py(py::module_ &m_) {

    py::class_<FunRange, std::shared_ptr<FunRange>>(m_, "FunRange");

    py::class_<Fun<double>, std::shared_ptr<Fun<double>>>(m_, "FunDouble");

    py::class_<Test, std::shared_ptr<Test>>(m_, "Test");

    py::class_<PrimitiveRef<double>, std::shared_ptr<PrimitiveRef<double>>>(m_, "PrimitiveRefDouble");

    py::class_<MyVector<3>, std::shared_ptr<MyVector<3>>>(m_, "MyVector3");

    py::class_<MyVector<12>, std::shared_ptr<MyVector<12>>>(m_, "MyVector12");

    py::class_<MultipleTemplates<int, double>, std::shared_ptr<MultipleTemplates<int, double>>>(m_, "MultipleTemplatesIntDouble");

    py::class_<MultipleTemplates<int, float>, std::shared_ptr<MultipleTemplates<int, float>>>(m_, "MultipleTemplatesIntFloat");

    py::class_<ForwardKinematics, std::shared_ptr<ForwardKinematics>>(m_, "ForwardKinematics");

    py::class_<TemplatedConstructor, std::shared_ptr<TemplatedConstructor>>(m_, "TemplatedConstructor");

    py::class_<FastSet, std::shared_ptr<FastSet>>(m_, "FastSet");

    py::class_<HessianFactor, gtsam::GaussianFactor, std::shared_ptr<HessianFactor>>(m_, "HessianFactor");

    py::class_<SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::SmartProjectionFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, std::shared_ptr<SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>>>(m_, "SmartProjectionRigFactorPinholeCameraCal3_S2");

    py::class_<MyFactor<gtsam::Pose2, gtsam::Matrix>, std::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>>>(m_, "MyFactorPosePoint2");

    py::class_<SuperCoolFactor<gtsam::Pose3>, std::shared_ptr<SuperCoolFactor<gtsam::Pose3>>>(m_, "SuperCoolFactorPose3");
}

void gtwrap_bind_class_py(py::module_ &m_) {
#include "python/specializations.h"

    auto gtwrap_class_m__FunRange = py::reinterpret_borrow<py::class_<FunRange, std::shared_ptr<FunRange>>>(m_.attr("FunRange"));
    gtwrap_class_m__FunRange
        .def(py::init<>())
        .def("range",static_cast<FunRange (FunRange::*)(double)>(&FunRange::range), gtwrap::internal::py_arg<double>("d"))
        .def_static("create",static_cast<FunRange (*)()>(&FunRange::create));

    auto gtwrap_class_m__FunDouble = py::reinterpret_borrow<py::class_<Fun<double>, std::shared_ptr<Fun<double>>>>(m_.attr("FunDouble"));
    gtwrap_class_m__FunDouble
        .def("templatedMethodString",[](Fun<double>* self, double d, string t){return self->templatedMethod<string>(d, t);}, gtwrap::internal::py_arg<double>("d"), gtwrap::internal::py_arg<string>("t"))
        .def("multiTemplatedMethodStringSize_t",[](Fun<double>* self, double d, string t, size_t u){return self->multiTemplatedMethod<string,size_t>(d, t, u);}, gtwrap::internal::py_arg<double>("d"), gtwrap::internal::py_arg<string>("t"), gtwrap::internal::py_arg<size_t>("u"))
        .def("sets",static_cast<std::map<double, Fun<double>::double> (Fun<double>::*)()>(&Fun<double>::sets))
        .def_static("staticMethodWithThis",static_cast<Fun<double> (*)()>(&Fun<double>::staticMethodWithThis))
        .def_static("templatedStaticMethodInt",[](const int& m){return Fun<double>::templatedStaticMethod<int>(m);}, gtwrap::internal::py_arg<const int&>("m"));

    auto gtwrap_class_m__Test = py::reinterpret_borrow<py::class_<Test, std::shared_ptr<Test>>>(m_.attr("Test"));
    gtwrap_class_m__Test
        .def(py::init<>())
        .def(py::init<double, const gtsam::Matrix&>(), gtwrap::internal::py_arg<double>("a"), gtwrap::internal::py_arg<const gtsam::Matrix&>("b"))
        .def("return_pair",static_cast<std::pair<gtsam::Vector,gtsam::Matrix> (Test::*)(const gtsam::Vector&, const gtsam::Matrix&) const>(&Test::return_pair), gtwrap::internal::py_arg<const gtsam::Vector&>("v"), gtwrap::internal::py_arg<const gtsam::Matrix&>("A"))
        .def("return_pair",static_cast<std::pair<gtsam::Vector,gtsam::Matrix> (Test::*)(const gtsam::Vector&) const>(&Test::return_pair), gtwrap::internal::py_arg<const gtsam::Vector&>("v"))
        .def("return_bool",static_cast<bool (Test::*)(bool) const>(&Test::return_bool), gtwrap::internal::py_arg<bool>("value"))
        .def("return_size_t",static_cast<size_t (Test::*)(size_t) const>(&Test::return_size_t), gtwrap::internal::py_arg<size_t>("value"))
        .def("return_int",static_cast<int (Test::*)(int) const>(&Test::return_int), gtwrap::internal::py_arg<int>("value"))
        .def("return_double",static_cast<double (Test::*)(double) const>(&Test::return_double), gtwrap::internal::py_arg<double>("value"))
        .def("return_string",static_cast<string (Test::*)(string) const>(&Test::return_string), gtwrap::internal::py_arg<string>("value"))
        .def("return_vector1",static_cast<gtsam::Vector (Test::*)(const gtsam::Vector&) const>(&Test::return_vector1), gtwrap::internal::py_arg<const gtsam::Vector&>("value"))
        .def("return_matrix1",static_cast<gtsam::Matrix (Test::*)(const gtsam::Matrix&) const>(&Test::return_matrix1), gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("return_vector2",static_cast<gtsam::Vector (Test::*)(const gtsam::Vector&) const>(&Test::return_vector2), gtwrap::internal::py_arg<const gtsam::Vector&>("value"))
        .def("return_matrix2",static_cast<gtsam::Matrix (Test::*)(const gtsam::Matrix&) const>(&Test::return_matrix2), gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("return_vector2",static_cast<const gtsam::Vector& (Test::*)(const gtsam::Vector&) const>(&Test::return_vector2), py::return_value_policy::reference_internal, gtwrap::internal::py_arg<const gtsam::Vector&>("value"))
        .def("return_matrix2",static_cast<const gtsam::Matrix& (Test::*)(const gtsam::Matrix&) const>(&Test::return_matrix2), py::return_value_policy::reference_internal, gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("arg_EigenConstRef",static_cast<void (Test::*)(const gtsam::Matrix&) const>(&Test::arg_EigenConstRef), gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("push_back",static_cast<void (Test::*)(gtsam::Key)>(&Test::push_back), gtwrap::internal::py_arg<gtsam::Key>("key"))
        .def("return_field",static_cast<bool (Test::*)(const Test&) const>(&Test::return_field), gtwrap::internal::py_arg<const Test&>("t"))
        .def("return_TestPtr",static_cast<std::shared_ptr<Test> (Test::*)(const std::shared_ptr<Test>) const>(&Test::return_TestPtr), gtwrap::internal::py_arg<const std::shared_ptr<Test>>("value"))
        .def("return_Test",static_cast<Test (Test::*)(std::shared_ptr<Test>) const>(&Test::return_Test), gtwrap::internal::py_arg<std::shared_ptr<Test>>("value"))
        .def("return_Point2Ptr",static_cast<std::shared_ptr<gtsam::Point2> (Test::*)(bool) const>(&Test::return_Point2Ptr), gtwrap::internal::py_arg<bool>("value"))
        .def("create_ptrs",static_cast<std::pair<std::shared_ptr<Test>,std::shared_ptr<Test>> (Test::*)() const>(&Test::create_ptrs))
        .def("create_MixedPtrs",static_cast<std::pair<Test,std::shared_ptr<Test>> (Test::*)() const>(&Test::create_MixedPtrs))
        .def("return_ptrs",static_cast<std::pair<std::shared_ptr<Test>,std::shared_ptr<Test>> (Test::*)(std::shared_ptr<Test>, std::shared_ptr<Test>) const>(&Test::return_ptrs), gtwrap::internal::py_arg<std::shared_ptr<Test>>("p1"), gtwrap::internal::py_arg<std::shared_ptr<Test>>("p2"))
        .def("print",[](Test* self){ py::scoped_ostream_redirect output; self->print();})
        .def("__repr__",
                    [](const Test& self){
                        gtsam::RedirectCout redirect;
                        self.print();
                        return redirect.str();
                    })
        .def("lambda_",static_cast<void (Test::*)() const>(&Test::lambda))
        .def("set_container",static_cast<void (Test::*)(std::vector<testing::Test>)>(&Test::set_container), gtwrap::internal::py_arg<std::vector<testing::Test>>("container"))
        .def("set_container",static_cast<void (Test::*)(std::vector<std::shared_ptr<testing::Test>>)>(&Test::set_container), gtwrap::internal::py_arg<std::vector<std::shared_ptr<testing::Test>>>("container"))
        .def("set_container",static_cast<void (Test::*)(std::vector<testing::Test&>)>(&Test::set_container), gtwrap::internal::py_arg<std::vector<testing::Test&>>("container"))
        .def("get_container",static_cast<std::vector<std::shared_ptr<testing::Test>> (Test::*)() const>(&Test::get_container))
        .def("_repr_markdown_",static_cast<string (Test::*)(const gtsam::KeyFormatter&) const>(&Test::markdown), gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def_readwrite("model_ptr", &Test::model_ptr)
        .def_readwrite("value", &Test::value)
        .def_readwrite("name", &Test::name);

    auto gtwrap_class_m__PrimitiveRefDouble = py::reinterpret_borrow<py::class_<PrimitiveRef<double>, std::shared_ptr<PrimitiveRef<double>>>>(m_.attr("PrimitiveRefDouble"));
    gtwrap_class_m__PrimitiveRefDouble
        .def(py::init<>())
        .def_static("Brutal",static_cast<PrimitiveRef<double> (*)(const double&)>(&PrimitiveRef<double>::Brutal), gtwrap::internal::py_arg<const double&>("t"));

    auto gtwrap_class_m__MyVector3 = py::reinterpret_borrow<py::class_<MyVector<3>, std::shared_ptr<MyVector<3>>>>(m_.attr("MyVector3"));
    gtwrap_class_m__MyVector3
        .def(py::init<>());

    auto gtwrap_class_m__MyVector12 = py::reinterpret_borrow<py::class_<MyVector<12>, std::shared_ptr<MyVector<12>>>>(m_.attr("MyVector12"));
    gtwrap_class_m__MyVector12
        .def(py::init<>());

    auto gtwrap_class_m__ForwardKinematics = py::reinterpret_borrow<py::class_<ForwardKinematics, std::shared_ptr<ForwardKinematics>>>(m_.attr("ForwardKinematics"));
    gtwrap_class_m__ForwardKinematics
        .def(py::init<const gtdynamics::Robot&, const string&, const string&, const gtsam::Values&, const gtsam::Pose3&>(), gtwrap::internal::py_arg<const gtdynamics::Robot&>("robot"), gtwrap::internal::py_arg<const string&>("start_link_name"), gtwrap::internal::py_arg<const string&>("end_link_name"), gtwrap::internal::py_arg<const gtsam::Values&>("joint_angles"), gtwrap::internal::py_arg<const gtsam::Pose3&>("l2Tp") = gtsam::Pose3());

    auto gtwrap_class_m__TemplatedConstructor = py::reinterpret_borrow<py::class_<TemplatedConstructor, std::shared_ptr<TemplatedConstructor>>>(m_.attr("TemplatedConstructor"));
    gtwrap_class_m__TemplatedConstructor
        .def(py::init<>())
        .def(py::init<const string&>(), gtwrap::internal::py_arg<const string&>("arg"))
        .def(py::init<const int&>(), gtwrap::internal::py_arg<const int&>("arg"))
        .def(py::init<const double&>(), gtwrap::internal::py_arg<const double&>("arg"));

    auto gtwrap_class_m__FastSet = py::reinterpret_borrow<py::class_<FastSet, std::shared_ptr<FastSet>>>(m_.attr("FastSet"));
    gtwrap_class_m__FastSet
        .def(py::init<>())
        .def("__len__",[](FastSet* self){return std::distance(self->begin(), self->end());})
        .def("__contains__",[](FastSet* self, size_t key){return std::find(self->begin(), self->end(), key) != self->end();}, gtwrap::internal::py_arg<size_t>("key"))
        .def("__iter__",[](FastSet* self){return py::make_iterator(self->begin(), self->end());});

    auto gtwrap_class_m__HessianFactor = py::reinterpret_borrow<py::class_<HessianFactor, gtsam::GaussianFactor, std::shared_ptr<HessianFactor>>>(m_.attr("HessianFactor"));
    gtwrap_class_m__HessianFactor
        .def(py::init<const gtsam::KeyVector&, const std::vector<gtsam::Matrix>&, const std::vector<gtsam::Vector>&, double>(), gtwrap::internal::py_arg<const gtsam::KeyVector&>("js"), gtwrap::internal::py_arg<const std::vector<gtsam::Matrix>&>("Gs"), gtwrap::internal::py_arg<const std::vector<gtsam::Vector>&>("gs"), gtwrap::internal::py_arg<double>("f"));

    auto gtwrap_class_m__SmartProjectionRigFactorPinholeCameraCal3_S2 = py::reinterpret_borrow<py::class_<SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::SmartProjectionFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, std::shared_ptr<SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>>>>(m_.attr("SmartProjectionRigFactorPinholeCameraCal3_S2"));
    gtwrap_class_m__SmartProjectionRigFactorPinholeCameraCal3_S2
        .def("add",static_cast<void (SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>::*)(const gtsam::PinholeCamera<gtsam::Cal3_S2>::Measurement&, const gtsam::Key&, const size_t&)>(&SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>::add), gtwrap::internal::py_arg<const gtsam::PinholeCamera<gtsam::Cal3_S2>::Measurement&>("measured"), gtwrap::internal::py_arg<const gtsam::Key&>("poseKey"), gtwrap::internal::py_arg<const size_t&>("cameraId") = 0);

    auto gtwrap_class_m__MyFactorPosePoint2 = py::reinterpret_borrow<py::class_<MyFactor<gtsam::Pose2, gtsam::Matrix>, std::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>>>>(m_.attr("MyFactorPosePoint2"));
    gtwrap_class_m__MyFactorPosePoint2
        .def(py::init<size_t, size_t, double, const std::shared_ptr<gtsam::noiseModel::Base>>(), gtwrap::internal::py_arg<size_t>("key1"), gtwrap::internal::py_arg<size_t>("key2"), gtwrap::internal::py_arg<double>("measured"), gtwrap::internal::py_arg<const std::shared_ptr<gtsam::noiseModel::Base>>("noiseModel"))
        .def("print",[](MyFactor<gtsam::Pose2, gtsam::Matrix>* self, const string& s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, gtwrap::internal::py_arg<const string&>("s") = "factor: ", gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const MyFactor<gtsam::Pose2, gtsam::Matrix>& self, const string& s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, gtwrap::internal::py_arg<const string&>("s") = "factor: ", gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter);

}

PYBIND11_MODULE(class_py, m_) {
    m_.doc() = "pybind11 wrapper of class_py";

gtwrap_declare_class_py(m_);
gtwrap_bind_class_py(m_);
}
