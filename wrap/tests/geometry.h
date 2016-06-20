  // comments!

class VectorNotEigen;
virtual class ns::OtherClass;

namespace gtsam {

class Point2 {
 Point2();
 Point2(double x, double y);
 double x() const;
 double y() const;
 int dim() const;
 char returnChar() const;
 void argChar(char a) const;
 void argUChar(unsigned char a) const;
 void eigenArguments(Vector v, Matrix m) const;
 VectorNotEigen vectorConfusion();

 void serializable() const; // Sets flag and creates export, but does not make serialization functions
};

class Point3 {
  Point3(double x, double y, double z);
  double norm() const;

  // static functions - use static keyword and uppercase
  static double staticFunction();
  static gtsam::Point3 StaticFunctionRet(double z);

  // enabling serialization functionality
  void serialize() const; // Just triggers a flag internally and removes actual function
};

}
// another comment

// another comment

/**
 * A multi-line comment!
 */

// An include! Can go anywhere outside of a class, in any order
#include <folder/path/to/Test.h>

class Test {

  /* a comment! */
  // another comment
  Test();

  pair<Vector,Matrix> return_pair (Vector v, Matrix A) const; // intentionally the first method

  bool   return_bool   (bool   value) const; // comment after a line!
  size_t return_size_t (size_t value) const;
  int    return_int    (int    value) const;
  double return_double (double value) const;

  Test(double a, Matrix b); // a constructor in the middle of a class

  // comments in the middle!

  // (more) comments in the middle!

  string return_string (string value) const;
  Vector return_vector1(Vector value) const;
  Matrix return_matrix1(Matrix value) const;
  Vector return_vector2(Vector value) const;
  Matrix return_matrix2(Matrix value) const;
  void arg_EigenConstRef(const Matrix& value) const;

  bool return_field(const Test& t) const;

  Test* return_TestPtr(Test* value) const;
  Test  return_Test(Test* value) const;

  gtsam::Point2* return_Point2Ptr(bool value) const;

  pair<Test*,Test*> create_ptrs () const;
  pair<Test ,Test*> create_MixedPtrs () const;
  pair<Test*,Test*> return_ptrs (Test* p1, Test* p2) const;

  void print() const;

  // comments at the end!

  // even more comments at the end!
};


Vector aGlobalFunction();

// An overloaded global function
Vector overloadedGlobalFunction(int a);
Vector overloadedGlobalFunction(int a, double b);

// A base class
virtual class MyBase {

};

// A templated class
template<T = {gtsam::Point2, Matrix}>
virtual class MyTemplate : MyBase {
  MyTemplate();

  template<ARG = {gtsam::Point2, gtsam::Point3, Vector, Matrix}>
  ARG templatedMethod(const ARG& t);

  // Stress test templates and pointer combinations
  void accept_T(const T& value) const;
  void accept_Tptr(T* value) const;
  T* return_Tptr(T* value) const;
  T  return_T(T* value) const;
  pair<T*,T*> create_ptrs () const;
  pair<T ,T*> create_MixedPtrs () const;
  pair<T*,T*> return_ptrs (T* p1, T* p2) const;
};

// A doubly templated class
template<POSE, POINT>
class MyFactor {
  MyFactor(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel);
};

// and a typedef specializing it
typedef MyFactor<gtsam::Pose2, Matrix> MyFactorPosePoint2;

// A class with integer template arguments
template<N = {3,12}>
class MyVector {
  MyVector();
};

// comments at the end!

// even more comments at the end!
