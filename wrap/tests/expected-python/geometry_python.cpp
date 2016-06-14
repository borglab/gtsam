#include <boost/python.hpp>

using namespace boost::python;
BOOST_PYTHON_MODULE(geometry)
{
class_<Point2>("Point2")
  .def("Point2", &Point2::Point2);
  .def("argChar", &Point2::argChar);
  .def("argUChar", &Point2::argUChar);
  .def("dim", &Point2::dim);
  .def("eigenArguments", &Point2::eigenArguments);
  .def("returnChar", &Point2::returnChar);
  .def("vectorConfusion", &Point2::vectorConfusion);
  .def("x", &Point2::x);
  .def("y", &Point2::y);
;

class_<Point3>("Point3")
  .def("Point3", &Point3::Point3);
  .def("StaticFunctionRet", &Point3::StaticFunctionRet);
  .def("staticFunction", &Point3::staticFunction);
  .def("norm", &Point3::norm);
;

class_<Test>("Test")
  .def("Test", &Test::Test);
  .def("arg_EigenConstRef", &Test::arg_EigenConstRef);
  .def("create_MixedPtrs", &Test::create_MixedPtrs);
  .def("create_ptrs", &Test::create_ptrs);
  .def("print", &Test::print);
  .def("return_Point2Ptr", &Test::return_Point2Ptr);
  .def("return_Test", &Test::return_Test);
  .def("return_TestPtr", &Test::return_TestPtr);
  .def("return_bool", &Test::return_bool);
  .def("return_double", &Test::return_double);
  .def("return_field", &Test::return_field);
  .def("return_int", &Test::return_int);
  .def("return_matrix1", &Test::return_matrix1);
  .def("return_matrix2", &Test::return_matrix2);
  .def("return_pair", &Test::return_pair);
  .def("return_ptrs", &Test::return_ptrs);
  .def("return_size_t", &Test::return_size_t);
  .def("return_string", &Test::return_string);
  .def("return_vector1", &Test::return_vector1);
  .def("return_vector2", &Test::return_vector2);
;

class_<MyBase>("MyBase")
  .def("MyBase", &MyBase::MyBase);
;

class_<MyTemplatePoint2>("MyTemplatePoint2")
  .def("MyTemplatePoint2", &MyTemplatePoint2::MyTemplatePoint2);
  .def("accept_T", &MyTemplatePoint2::accept_T);
  .def("accept_Tptr", &MyTemplatePoint2::accept_Tptr);
  .def("create_MixedPtrs", &MyTemplatePoint2::create_MixedPtrs);
  .def("create_ptrs", &MyTemplatePoint2::create_ptrs);
  .def("return_T", &MyTemplatePoint2::return_T);
  .def("return_Tptr", &MyTemplatePoint2::return_Tptr);
  .def("return_ptrs", &MyTemplatePoint2::return_ptrs);
  .def("templatedMethod", &MyTemplatePoint2::templatedMethod);
  .def("templatedMethod", &MyTemplatePoint2::templatedMethod);
  .def("templatedMethod", &MyTemplatePoint2::templatedMethod);
  .def("templatedMethod", &MyTemplatePoint2::templatedMethod);
;

class_<MyTemplateMatrix>("MyTemplateMatrix")
  .def("MyTemplateMatrix", &MyTemplateMatrix::MyTemplateMatrix);
  .def("accept_T", &MyTemplateMatrix::accept_T);
  .def("accept_Tptr", &MyTemplateMatrix::accept_Tptr);
  .def("create_MixedPtrs", &MyTemplateMatrix::create_MixedPtrs);
  .def("create_ptrs", &MyTemplateMatrix::create_ptrs);
  .def("return_T", &MyTemplateMatrix::return_T);
  .def("return_Tptr", &MyTemplateMatrix::return_Tptr);
  .def("return_ptrs", &MyTemplateMatrix::return_ptrs);
  .def("templatedMethod", &MyTemplateMatrix::templatedMethod);
  .def("templatedMethod", &MyTemplateMatrix::templatedMethod);
  .def("templatedMethod", &MyTemplateMatrix::templatedMethod);
  .def("templatedMethod", &MyTemplateMatrix::templatedMethod);
;

class_<MyVector3>("MyVector3")
  .def("MyVector3", &MyVector3::MyVector3);
;

class_<MyVector12>("MyVector12")
  .def("MyVector12", &MyVector12::MyVector12);
;

class_<MyFactorPosePoint2>("MyFactorPosePoint2")
  .def("MyFactorPosePoint2", &MyFactorPosePoint2::MyFactorPosePoint2);
;

def("aGlobalFunction", aGlobalFunction);
def("overloadedGlobalFunction", overloadedGlobalFunction);
}
