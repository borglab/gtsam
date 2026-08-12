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

PYBIND11_MODULE(class_py, m_) {
    m_.doc() = "pybind11 wrapper of class_py";


    py::class_<FunRange, std::shared_ptr<FunRange>>(m_, "FunRange")
        .def(py::init<>())
        .def("range",[](FunRange* self, double d){return self->range(d);}, gtwrap::internal::py_arg<double>("d"))
        .def_static("create",[](){return FunRange::create();});

    py::class_<Fun<double>, std::shared_ptr<Fun<double>>>(m_, "FunDouble")
        .def("templatedMethodString",[](Fun<double>* self, double d, string t){return self->templatedMethod<string>(d, t);}, gtwrap::internal::py_arg<double>("d"), gtwrap::internal::py_arg<string>("t"))
        .def("multiTemplatedMethodStringSize_t",[](Fun<double>* self, double d, string t, size_t u){return self->multiTemplatedMethod<string,size_t>(d, t, u);}, gtwrap::internal::py_arg<double>("d"), gtwrap::internal::py_arg<string>("t"), gtwrap::internal::py_arg<size_t>("u"))
        .def("sets",[](Fun<double>* self){return self->sets();})
        .def_static("staticMethodWithThis",[](){return Fun<double>::staticMethodWithThis();})
        .def_static("templatedStaticMethodInt",[](const int& m){return Fun<double>::templatedStaticMethod<int>(m);}, gtwrap::internal::py_arg<const int&>("m"));

    py::class_<Test, std::shared_ptr<Test>>(m_, "Test")
        .def(py::init<>())
        .def(py::init<double, const gtsam::Matrix&>(), gtwrap::internal::py_arg<double>("a"), gtwrap::internal::py_arg<const gtsam::Matrix&>("b"))
        .def("return_pair",[](Test* self, const gtsam::Vector& v, const gtsam::Matrix& A){return self->return_pair(v, A);}, gtwrap::internal::py_arg<const gtsam::Vector&>("v"), gtwrap::internal::py_arg<const gtsam::Matrix&>("A"))
        .def("return_pair",[](Test* self, const gtsam::Vector& v){return self->return_pair(v);}, gtwrap::internal::py_arg<const gtsam::Vector&>("v"))
        .def("return_bool",[](Test* self, bool value){return self->return_bool(value);}, gtwrap::internal::py_arg<bool>("value"))
        .def("return_size_t",[](Test* self, size_t value){return self->return_size_t(value);}, gtwrap::internal::py_arg<size_t>("value"))
        .def("return_int",[](Test* self, int value){return self->return_int(value);}, gtwrap::internal::py_arg<int>("value"))
        .def("return_double",[](Test* self, double value){return self->return_double(value);}, gtwrap::internal::py_arg<double>("value"))
        .def("return_string",[](Test* self, string value){return self->return_string(value);}, gtwrap::internal::py_arg<string>("value"))
        .def("return_vector1",[](Test* self, const gtsam::Vector& value){return self->return_vector1(value);}, gtwrap::internal::py_arg<const gtsam::Vector&>("value"))
        .def("return_matrix1",[](Test* self, const gtsam::Matrix& value){return self->return_matrix1(value);}, gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("return_vector2",[](Test* self, const gtsam::Vector& value){return self->return_vector2(value);}, gtwrap::internal::py_arg<const gtsam::Vector&>("value"))
        .def("return_matrix2",[](Test* self, const gtsam::Matrix& value){return self->return_matrix2(value);}, gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("return_vector2",[](Test* self, const gtsam::Vector& value) -> const auto&{return self->return_vector2(value);}, py::return_value_policy::reference_internal, gtwrap::internal::py_arg<const gtsam::Vector&>("value"))
        .def("return_matrix2",[](Test* self, const gtsam::Matrix& value) -> const auto&{return self->return_matrix2(value);}, py::return_value_policy::reference_internal, gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("arg_EigenConstRef",[](Test* self, const gtsam::Matrix& value){ self->arg_EigenConstRef(value);}, gtwrap::internal::py_arg<const gtsam::Matrix&>("value"))
        .def("push_back",[](Test* self, gtsam::Key key){ self->push_back(key);}, gtwrap::internal::py_arg<gtsam::Key>("key"))
        .def("return_field",[](Test* self, const Test& t){return self->return_field(t);}, gtwrap::internal::py_arg<const Test&>("t"))
        .def("return_TestPtr",[](Test* self, const std::shared_ptr<Test> value){return self->return_TestPtr(value);}, gtwrap::internal::py_arg<const std::shared_ptr<Test>>("value"))
        .def("return_Test",[](Test* self, std::shared_ptr<Test> value){return self->return_Test(value);}, gtwrap::internal::py_arg<std::shared_ptr<Test>>("value"))
        .def("return_Point2Ptr",[](Test* self, bool value){return self->return_Point2Ptr(value);}, gtwrap::internal::py_arg<bool>("value"))
        .def("create_ptrs",[](Test* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](Test* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](Test* self, std::shared_ptr<Test> p1, std::shared_ptr<Test> p2){return self->return_ptrs(p1, p2);}, gtwrap::internal::py_arg<std::shared_ptr<Test>>("p1"), gtwrap::internal::py_arg<std::shared_ptr<Test>>("p2"))
        .def("print",[](Test* self){ py::scoped_ostream_redirect output; self->print();})
        .def("__repr__",
                    [](const Test& self){
                        gtsam::RedirectCout redirect;
                        self.print();
                        return redirect.str();
                    })
        .def("lambda_",[](Test* self){ self->lambda();})
        .def("set_container",[](Test* self, std::vector<testing::Test> container){ self->set_container(container);}, gtwrap::internal::py_arg<std::vector<testing::Test>>("container"))
        .def("set_container",[](Test* self, std::vector<std::shared_ptr<testing::Test>> container){ self->set_container(container);}, gtwrap::internal::py_arg<std::vector<std::shared_ptr<testing::Test>>>("container"))
        .def("set_container",[](Test* self, std::vector<testing::Test&> container){ self->set_container(container);}, gtwrap::internal::py_arg<std::vector<testing::Test&>>("container"))
        .def("get_container",[](Test* self){return self->get_container();})
        .def("_repr_markdown_",[](Test* self, const gtsam::KeyFormatter& keyFormatter){return self->markdown(keyFormatter);}, gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def_readwrite("model_ptr", &Test::model_ptr)
        .def_readwrite("value", &Test::value)
        .def_readwrite("name", &Test::name);

    py::class_<PrimitiveRef<double>, std::shared_ptr<PrimitiveRef<double>>>(m_, "PrimitiveRefDouble")
        .def(py::init<>())
        .def_static("Brutal",[](const double& t){return PrimitiveRef<double>::Brutal(t);}, gtwrap::internal::py_arg<const double&>("t"));

    py::class_<MyVector<3>, std::shared_ptr<MyVector<3>>>(m_, "MyVector3")
        .def(py::init<>());

    py::class_<MyVector<12>, std::shared_ptr<MyVector<12>>>(m_, "MyVector12")
        .def(py::init<>());

    py::class_<MultipleTemplates<int, double>, std::shared_ptr<MultipleTemplates<int, double>>>(m_, "MultipleTemplatesIntDouble");

    py::class_<MultipleTemplates<int, float>, std::shared_ptr<MultipleTemplates<int, float>>>(m_, "MultipleTemplatesIntFloat");

    py::class_<ForwardKinematics, std::shared_ptr<ForwardKinematics>>(m_, "ForwardKinematics")
        .def(py::init<const gtdynamics::Robot&, const string&, const string&, const gtsam::Values&, const gtsam::Pose3&>(), gtwrap::internal::py_arg<const gtdynamics::Robot&>("robot"), gtwrap::internal::py_arg<const string&>("start_link_name"), gtwrap::internal::py_arg<const string&>("end_link_name"), gtwrap::internal::py_arg<const gtsam::Values&>("joint_angles"), gtwrap::internal::py_arg<const gtsam::Pose3&>("l2Tp") = gtsam::Pose3());

    py::class_<TemplatedConstructor, std::shared_ptr<TemplatedConstructor>>(m_, "TemplatedConstructor")
        .def(py::init<>())
        .def(py::init<const string&>(), gtwrap::internal::py_arg<const string&>("arg"))
        .def(py::init<const int&>(), gtwrap::internal::py_arg<const int&>("arg"))
        .def(py::init<const double&>(), gtwrap::internal::py_arg<const double&>("arg"));

    py::class_<FastSet, std::shared_ptr<FastSet>>(m_, "FastSet")
        .def(py::init<>())
        .def("__len__",[](FastSet* self){return std::distance(self->begin(), self->end());})
        .def("__contains__",[](FastSet* self, size_t key){return std::find(self->begin(), self->end(), key) != self->end();}, gtwrap::internal::py_arg<size_t>("key"))
        .def("__iter__",[](FastSet* self){return py::make_iterator(self->begin(), self->end());});

    py::class_<HessianFactor, gtsam::GaussianFactor, std::shared_ptr<HessianFactor>>(m_, "HessianFactor")
        .def(py::init<const gtsam::KeyVector&, const std::vector<gtsam::Matrix>&, const std::vector<gtsam::Vector>&, double>(), gtwrap::internal::py_arg<const gtsam::KeyVector&>("js"), gtwrap::internal::py_arg<const std::vector<gtsam::Matrix>&>("Gs"), gtwrap::internal::py_arg<const std::vector<gtsam::Vector>&>("gs"), gtwrap::internal::py_arg<double>("f"));

    py::class_<SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, gtsam::SmartProjectionFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>, std::shared_ptr<SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>>>(m_, "SmartProjectionRigFactorPinholeCameraCal3_S2")
        .def("add",[](SmartProjectionRigFactor<gtsam::PinholeCamera<gtsam::Cal3_S2>>* self, const gtsam::PinholeCamera<gtsam::Cal3_S2>::Measurement& measured, const gtsam::Key& poseKey, const size_t& cameraId){ self->add(measured, poseKey, cameraId);}, gtwrap::internal::py_arg<const gtsam::PinholeCamera<gtsam::Cal3_S2>::Measurement&>("measured"), gtwrap::internal::py_arg<const gtsam::Key&>("poseKey"), gtwrap::internal::py_arg<const size_t&>("cameraId") = 0);

    py::class_<MyFactor<gtsam::Pose2, gtsam::Matrix>, std::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>>>(m_, "MyFactorPosePoint2")
        .def(py::init<size_t, size_t, double, const std::shared_ptr<gtsam::noiseModel::Base>>(), gtwrap::internal::py_arg<size_t>("key1"), gtwrap::internal::py_arg<size_t>("key2"), gtwrap::internal::py_arg<double>("measured"), gtwrap::internal::py_arg<const std::shared_ptr<gtsam::noiseModel::Base>>("noiseModel"))
        .def("print",[](MyFactor<gtsam::Pose2, gtsam::Matrix>* self, const string& s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, gtwrap::internal::py_arg<const string&>("s") = "factor: ", gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const MyFactor<gtsam::Pose2, gtsam::Matrix>& self, const string& s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, gtwrap::internal::py_arg<const string&>("s") = "factor: ", gtwrap::internal::py_arg<const gtsam::KeyFormatter&>("keyFormatter") = gtsam::DefaultKeyFormatter);

    py::class_<SuperCoolFactor<gtsam::Pose3>, std::shared_ptr<SuperCoolFactor<gtsam::Pose3>>>(m_, "SuperCoolFactorPose3");

#include "python/specializations.h"

}

