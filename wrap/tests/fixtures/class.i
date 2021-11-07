class FunRange {
  FunRange();
  This range(double d);

  static This create();
};

template<M={double}>
class Fun {

  static This staticMethodWithThis();

  template<T={int}>
  static double templatedStaticMethod(const T& m);

  template<T={string}>
  This templatedMethod(double d, T t);

  template<T={string}, U={size_t}>
  This multiTemplatedMethod(double d, T t, U u);
};


// An include! Can go anywhere outside of a class, in any order
#include <folder/path/to/Test.h>

class Test {

  /* a comment! */
  // another comment
  Test();

  // Test a shared ptr property
  gtsam::noiseModel::Base* model_ptr;

  pair<Vector,Matrix> return_pair (Vector v, Matrix A) const; // intentionally the first method
  pair<Vector,Matrix> return_pair (Vector v) const; // overload

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

  Test* return_TestPtr(const Test* value) const;
  Test  return_Test(Test* value) const;

  gtsam::Point2* return_Point2Ptr(bool value) const;

  pair<Test*,Test*> create_ptrs () const;
  pair<Test ,Test*> create_MixedPtrs () const;
  pair<Test*,Test*> return_ptrs (Test* p1, Test* p2) const;

  // This should be callable as .print() in python
  void print() const;
  // Since this is a reserved keyword, it should be updated to `lambda_`
  void lambda() const;

  void set_container(std::vector<testing::Test> container);
  void set_container(std::vector<testing::Test*> container);
  void set_container(std::vector<testing::Test&> container);
  std::vector<testing::Test*> get_container() const;

  // comments at the end!

  // even more comments at the end!
};

virtual class ns::OtherClass;

// A doubly templated class
template<POSE, POINT>
class MyFactor {
  MyFactor(size_t key1, size_t key2, double measured, const gtsam::noiseModel::Base* noiseModel);
  void print(const string &s = "factor: ",
             const gtsam::KeyFormatter &keyFormatter = gtsam::DefaultKeyFormatter);
};

// and a typedef specializing it
typedef MyFactor<gtsam::Pose2, Matrix> MyFactorPosePoint2;

template<T = {double}>
class PrimitiveRef {
  PrimitiveRef();

  static This Brutal(const T& t);
};

// A class with integer template arguments
template<N = {3,12}>
class MyVector {
  MyVector();
};

// comments at the end!

// even more comments at the end!

// Class with multiple instantiated templates
template<T = {int}, U = {double, float}>
class MultipleTemplates {};

// Test for default args in constructor
class ForwardKinematics {
  ForwardKinematics(const gtdynamics::Robot& robot,
                    const string& start_link_name, const string& end_link_name,
                    const gtsam::Values& joint_angles,
                    const gtsam::Pose3& l2Tp = gtsam::Pose3());
};

// Test for templated constructor
class TemplatedConstructor {
  TemplatedConstructor();

  template<T={string, int, double}>
  TemplatedConstructor(const T& arg);
};


class SuperCoolFactor;
typedef SuperCoolFactor<gtsam::Pose3> SuperCoolFactorPose3;
