#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include "gtsam/nonlinear/utilities.h"  // for RedirectCout.

#include "folder/path/to/Test.h"




using namespace std;

namespace py = pybind11;

PYBIND11_MODULE(class_py, m_) {
    m_.doc() = "pybind11 wrapper of class_py";


    py::class_<FunRange, std::shared_ptr<FunRange>>(m_, "FunRange")
        .def(py::init<>())
        .def("range",[](FunRange* self, double d){return self->range(d);}, py::arg("d"))
        .def_static("create",[](){return FunRange::create();});

    py::class_<Fun<double>, std::shared_ptr<Fun<double>>>(m_, "FunDouble")
        .def("templatedMethodString",[](Fun<double>* self, double d, string t){return self->templatedMethod<string>(d, t);}, py::arg("d"), py::arg("t"))
        .def("multiTemplatedMethodStringSize_t",[](Fun<double>* self, double d, string t, size_t u){return self->multiTemplatedMethod<string,size_t>(d, t, u);}, py::arg("d"), py::arg("t"), py::arg("u"))
        .def("sets",[](Fun<double>* self){return self->sets();})
        .def_static("staticMethodWithThis",[](){return Fun<double>::staticMethodWithThis();})
        .def_static("templatedStaticMethodInt",[](const int& m){return Fun<double>::templatedStaticMethod<int>(m);}, py::arg("m"));

    py::class_<Test, std::shared_ptr<Test>>(m_, "Test")
        .def(py::init<>())
        .def(py::init<double, const gtsam::Matrix&>(), py::arg("a"), py::arg("b"))
        .def("return_pair",[](Test* self, const gtsam::Vector& v, const gtsam::Matrix& A){return self->return_pair(v, A);}, py::arg("v"), py::arg("A"))
        .def("return_pair",[](Test* self, const gtsam::Vector& v){return self->return_pair(v);}, py::arg("v"))
        .def("return_bool",[](Test* self, bool value){return self->return_bool(value);}, py::arg("value"))
        .def("return_size_t",[](Test* self, size_t value){return self->return_size_t(value);}, py::arg("value"))
        .def("return_int",[](Test* self, int value){return self->return_int(value);}, py::arg("value"))
        .def("return_double",[](Test* self, double value){return self->return_double(value);}, py::arg("value"))
        .def("return_string",[](Test* self, string value){return self->return_string(value);}, py::arg("value"))
        .def("return_vector1",[](Test* self, const gtsam::Vector& value){return self->return_vector1(value);}, py::arg("value"))
        .def("return_matrix1",[](Test* self, const gtsam::Matrix& value){return self->return_matrix1(value);}, py::arg("value"))
        .def("return_vector2",[](Test* self, const gtsam::Vector& value){return self->return_vector2(value);}, py::arg("value"))
        .def("return_matrix2",[](Test* self, const gtsam::Matrix& value){return self->return_matrix2(value);}, py::arg("value"))
        .def("arg_EigenConstRef",[](Test* self, const gtsam::Matrix& value){ self->arg_EigenConstRef(value);}, py::arg("value"))
        .def("return_field",[](Test* self, const Test& t){return self->return_field(t);}, py::arg("t"))
        .def("return_TestPtr",[](Test* self, const std::shared_ptr<Test> value){return self->return_TestPtr(value);}, py::arg("value"))
        .def("return_Test",[](Test* self, std::shared_ptr<Test> value){return self->return_Test(value);}, py::arg("value"))
        .def("return_Point2Ptr",[](Test* self, bool value){return self->return_Point2Ptr(value);}, py::arg("value"))
        .def("create_ptrs",[](Test* self){return self->create_ptrs();})
        .def("create_MixedPtrs",[](Test* self){return self->create_MixedPtrs();})
        .def("return_ptrs",[](Test* self, std::shared_ptr<Test> p1, std::shared_ptr<Test> p2){return self->return_ptrs(p1, p2);}, py::arg("p1"), py::arg("p2"))
        .def("print",[](Test* self){ py::scoped_ostream_redirect output; self->print();})
        .def("__repr__",
                    [](const Test& self){
                        gtsam::RedirectCout redirect;
                        self.print();
                        return redirect.str();
                    })
        .def("lambda_",[](Test* self){ self->lambda();})
        .def("set_container",[](Test* self, std::vector<testing::Test> container){ self->set_container(container);}, py::arg("container"))
        .def("set_container",[](Test* self, std::vector<std::shared_ptr<testing::Test>> container){ self->set_container(container);}, py::arg("container"))
        .def("set_container",[](Test* self, std::vector<testing::Test&> container){ self->set_container(container);}, py::arg("container"))
        .def("get_container",[](Test* self){return self->get_container();})
        .def("_repr_markdown_",[](Test* self, const gtsam::KeyFormatter& keyFormatter){return self->markdown(keyFormatter);}, py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def_readwrite("model_ptr", &Test::model_ptr)
        .def_readwrite("value", &Test::value)
        .def_readwrite("name", &Test::name);

    py::class_<PrimitiveRef<double>, std::shared_ptr<PrimitiveRef<double>>>(m_, "PrimitiveRefDouble")
        .def(py::init<>())
        .def_static("Brutal",[](const double& t){return PrimitiveRef<double>::Brutal(t);}, py::arg("t"));

    py::class_<MyVector<3>, std::shared_ptr<MyVector<3>>>(m_, "MyVector3")
        .def(py::init<>());

    py::class_<MyVector<12>, std::shared_ptr<MyVector<12>>>(m_, "MyVector12")
        .def(py::init<>());

    py::class_<MultipleTemplates<int, double>, std::shared_ptr<MultipleTemplates<int, double>>>(m_, "MultipleTemplatesIntDouble");

    py::class_<MultipleTemplates<int, float>, std::shared_ptr<MultipleTemplates<int, float>>>(m_, "MultipleTemplatesIntFloat");

    py::class_<ForwardKinematics, std::shared_ptr<ForwardKinematics>>(m_, "ForwardKinematics")
        .def(py::init<const gtdynamics::Robot&, const string&, const string&, const gtsam::Values&, const gtsam::Pose3&>(), py::arg("robot"), py::arg("start_link_name"), py::arg("end_link_name"), py::arg("joint_angles"), py::arg("l2Tp") = gtsam::Pose3());

    py::class_<TemplatedConstructor, std::shared_ptr<TemplatedConstructor>>(m_, "TemplatedConstructor")
        .def(py::init<>())
        .def(py::init<const string&>(), py::arg("arg"))
        .def(py::init<const int&>(), py::arg("arg"))
        .def(py::init<const double&>(), py::arg("arg"));

    py::class_<FastSet, std::shared_ptr<FastSet>>(m_, "FastSet")
        .def(py::init<>())
        .def("__len__",[](FastSet* self){return std::distance(self->begin(), self->end());})
        .def("__contains__",[](FastSet* self, size_t key){return std::find(self->begin(), self->end(), key) != self->end();}, py::arg("key"))
        .def("__iter__",[](FastSet* self){return py::make_iterator(self->begin(), self->end());});

    py::class_<HessianFactor, gtsam::GaussianFactor, std::shared_ptr<HessianFactor>>(m_, "HessianFactor")
        .def(py::init<const gtsam::KeyVector&, const std::vector<gtsam::Matrix>&, const std::vector<gtsam::Vector>&, double>(), py::arg("js"), py::arg("Gs"), py::arg("gs"), py::arg("f"));

    py::class_<MyFactor<gtsam::Pose2, gtsam::Matrix>, std::shared_ptr<MyFactor<gtsam::Pose2, gtsam::Matrix>>>(m_, "MyFactorPosePoint2")
        .def(py::init<size_t, size_t, double, const std::shared_ptr<gtsam::noiseModel::Base>>(), py::arg("key1"), py::arg("key2"), py::arg("measured"), py::arg("noiseModel"))
        .def("print",[](MyFactor<gtsam::Pose2, gtsam::Matrix>* self, const string& s, const gtsam::KeyFormatter& keyFormatter){ py::scoped_ostream_redirect output; self->print(s, keyFormatter);}, py::arg("s") = "factor: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter)
        .def("__repr__",
                    [](const MyFactor<gtsam::Pose2, gtsam::Matrix>& self, const string& s, const gtsam::KeyFormatter& keyFormatter){
                        gtsam::RedirectCout redirect;
                        self.print(s, keyFormatter);
                        return redirect.str();
                    }, py::arg("s") = "factor: ", py::arg("keyFormatter") = gtsam::DefaultKeyFormatter);

    py::class_<SuperCoolFactor<gtsam::Pose3>, std::shared_ptr<SuperCoolFactor<gtsam::Pose3>>>(m_, "SuperCoolFactorPose3");

#include "python/specializations.h"

}

