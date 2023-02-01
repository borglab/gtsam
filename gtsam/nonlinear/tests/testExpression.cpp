/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testExpression.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam/nonlinear/expressions.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Expression<Point3> Point3_;
typedef Expression<Pose3> Pose3_;
typedef Expression<Rot3> Rot3_;

/* ************************************************************************* */
template <class CAL>
Point2 uncalibrate(const CAL& K, const Point2& p, OptionalJacobian<2, 5> Dcal,
                   OptionalJacobian<2, 2> Dp) {
  return K.uncalibrate(p, Dcal, Dp);
}

static const Rot3 someR = Rot3::RzRyRx(1, 2, 3);

/* ************************************************************************* */
// Constant
TEST(Expression, Constant) {
  Rot3_ R(someR);
  Values values;
  Rot3 actual = R.value(values);
  EXPECT(assert_equal(someR, actual));
  EXPECT_LONGS_EQUAL(0, R.traceSize())
}

/* ************************************************************************* */
// Leaf
TEST(Expression, Leaf) {
  const Key key = 100;
  Rot3_ R(key);
  Values values;
  values.insert(key, someR);

  Rot3 actual2 = R.value(values);
  EXPECT(assert_equal(someR, actual2));
}

/* ************************************************************************* */
// Test the function `createUnknowns` to create many leaves at once.
TEST(Expression, Leaves) {
  Values values;
  const Point3 somePoint(1, 2, 3);
  values.insert(Symbol('p', 10), somePoint);
  std::vector<Point3_> pointExpressions = createUnknowns<Point3>(10, 'p', 1);
  EXPECT(assert_equal(somePoint, pointExpressions.back().value(values)));
}

/* ************************************************************************* */
// Unary(Leaf)
namespace unary {
Point2 f1(const Point3& p, OptionalJacobian<2, 3> H) {
  return Point2(0,0);
}
double f2(const Point3& p, OptionalJacobian<1, 3> H) {
  return 0.0;
}
Vector f3(const Point3& p, OptionalJacobian<Eigen::Dynamic, 3> H) {
  return p;
}
Point3_ pointExpression(1);
const set<Key> expected{1};
}  // namespace unary

// Create a unary expression that takes another expression as a single argument.
TEST(Expression, Unary1) {
  using namespace unary;
  Expression<Point2> unaryExpression(f1, pointExpression);
  EXPECT(expected == unaryExpression.keys());
}

// Check that also works with a scalar return value.
TEST(Expression, Unary2) {
  using namespace unary;
  Double_ unaryExpression(f2, pointExpression);
  EXPECT(expected == unaryExpression.keys());
}

// Unary(Leaf), dynamic
TEST(Expression, Unary3) {
  using namespace unary;
  // TODO(yetongumich): dynamic output arguments do not work yet!
  // Expression<Vector> unaryExpression(f3, pointExpression);
  // EXPECT(expected == unaryExpression.keys());
}

/* ************************************************************************* */
// Simple test class that implements the `VectorSpace` protocol.
class Class : public Point3 {
 public:
  enum {dimension = 3};
  using Point3::Point3;
  const Vector3& vector() const { return *this; }
  inline static Class Identity() { return Class(0,0,0); }
  double norm(OptionalJacobian<1,3> H = boost::none) const {
    return norm3(*this, H);
  }
  bool equals(const Class &q, double tol) const {
    return (std::abs(x() - q.x()) < tol && std::abs(y() - q.y()) < tol && std::abs(z() - q.z()) < tol);
  }
  void print(const string& s) const { cout << s << *this << endl;}
};

namespace gtsam {
template<> struct traits<Class> : public internal::VectorSpace<Class> {};
}

// Nullary Method
TEST(Expression, NullaryMethod) {
  // Create expression
  const Key key(67);
  Expression<Class> classExpression(key);

  // Make expression from a class method, note how it differs from the function
  // expressions by leading with the class expression in the constructor.
  Expression<double> norm_(classExpression, &Class::norm);

  // Create Values
  Values values;
  values.insert(key, Class(3, 4, 5));

  // Check dims as map
  std::map<Key, int> map;
  norm_.dims(map); // TODO(yetongumich): Change to google style pointer convention.
  LONGS_EQUAL(1, map.size());

  // Get value and Jacobians
  std::vector<Matrix> H(1);
  double actual = norm_.value(values, H);

  // Check all
  const double norm = sqrt(3*3 + 4*4 + 5*5);
  EXPECT(actual == norm);
  Matrix expected(1, 3);
  expected << 3.0 / norm, 4.0 / norm, 5.0 / norm;
  EXPECT(assert_equal(expected, H[0]));
}

/* ************************************************************************* */
// Binary(Leaf,Leaf)
namespace binary {
// Create leaves
double doubleF(const Pose3& pose,  //
               const Point3& point, OptionalJacobian<1, 6> H1, OptionalJacobian<1, 3> H2) {
  return 0.0;
}
Pose3_ x(1);
Point3_ p(2);
Point3_ p_cam(x, &Pose3::transformTo, p);
}

/* ************************************************************************* */
// Check that creating an expression to double compiles.
TEST(Expression, BinaryToDouble) {
  using namespace binary;
  Double_ p_cam(doubleF, x, p);
}

/* ************************************************************************* */
// Check keys of an expression created from class method.
TEST(Expression, BinaryKeys) {
  const set<Key> expected{1, 2};
  EXPECT(expected == binary::p_cam.keys());
}

/* ************************************************************************* */
// Check dimensions by calling `dims` method.
TEST(Expression, BinaryDimensions) {
  map<Key, int> actual, expected{{1, 6}, {2, 3}};
  binary::p_cam.dims(actual);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check dimensions of execution trace.
TEST(Expression, BinaryTraceSize) {
  typedef internal::BinaryExpression<Point3, Pose3, Point3> Binary;
  size_t expectedTraceSize = sizeof(Binary::Record);
  internal::upAlign(expectedTraceSize);
  EXPECT_LONGS_EQUAL(expectedTraceSize, binary::p_cam.traceSize());
}

/* ************************************************************************* */
// Binary(Leaf,Unary(Binary(Leaf,Leaf)))
namespace tree {
using namespace binary;
// Create leaves
Expression<Cal3_S2> K(3);

// Create expression tree
Point2 (*f)(const Point3&, OptionalJacobian<2, 3>) = &PinholeBase::Project;
Expression<Point2> projection(f, p_cam);
Expression<Point2> uv_hat(uncalibrate<Cal3_S2>, K, projection);
}

/* ************************************************************************* */
// keys
TEST(Expression, TreeKeys) {
  const set<Key> expected{1, 2, 3};
  EXPECT(expected == tree::uv_hat.keys());
}

/* ************************************************************************* */
// dimensions
TEST(Expression, TreeDimensions) {
  map<Key, int> actual, expected{{1, 6}, {2, 3}, {3, 5}};
  tree::uv_hat.dims(actual);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// TraceSize
TEST(Expression, TreeTraceSize) {
  typedef internal::BinaryExpression<Point3, Pose3, Point3> Binary1;
  EXPECT_LONGS_EQUAL(internal::upAligned(sizeof(Binary1::Record)), tree::p_cam.traceSize());

  typedef internal::UnaryExpression<Point2, Point3> Unary;
  EXPECT_LONGS_EQUAL(internal::upAligned(sizeof(Unary::Record)) + tree::p_cam.traceSize(),
                     tree::projection.traceSize());

  EXPECT_LONGS_EQUAL(0, tree::K.traceSize());

  typedef internal::BinaryExpression<Point2, Cal3_S2, Point2> Binary2;
  EXPECT_LONGS_EQUAL(internal::upAligned(sizeof(Binary2::Record)) + tree::K.traceSize() +
                         tree::projection.traceSize(),
                     tree::uv_hat.traceSize());
}

/* ************************************************************************* */
// Test compose operation with * operator.
TEST(Expression, compose1) {
  // Create expression
  Rot3_ R1(1), R2(2);
  Rot3_ R3 = R1 * R2;

  // Check keys
  const set<Key> expected{1, 2};
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test compose with arguments referring to the same rotation.
TEST(Expression, compose2) {
  // Create expression
  Rot3_ R1(1), R2(1);
  Rot3_ R3 = R1 * R2;

  // Check keys
  const set<Key> expected{1};
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test compose with one arguments referring to constant rotation.
TEST(Expression, compose3) {
  // Create expression
  Rot3_ R1(Rot3::Identity()), R2(3);
  Rot3_ R3 = R1 * R2;

  // Check keys
  const set<Key> expected{3};
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test compose with double type (should be multiplication).
TEST(Expression, compose4) {
  // Create expression
  gtsam::Key key = 1;
  Double_ R1(key), R2(key);
  Double_ R3 = R1 * R2;

  // Check keys
  const set<Key> expected{1};
  EXPECT(expected == R3.keys());
}

/* ************************************************************************* */
// Test with ternary function.
Rot3 composeThree(const Rot3& R1, const Rot3& R2, const Rot3& R3, OptionalJacobian<3, 3> H1,
                  OptionalJacobian<3, 3> H2, OptionalJacobian<3, 3> H3) {
  // return dummy derivatives (not correct, but that's ok for testing here)
  if (H1)
    *H1 = I_3x3;
  if (H2)
    *H2 = I_3x3;
  if (H3)
    *H3 = I_3x3;
  return R1 * (R2 * R3);
}

TEST(Expression, ternary) {
  // Create expression
  Rot3_ A(1), B(2), C(3);
  Rot3_ ABC(composeThree, A, B, C);

  // Check keys
  const set<Key> expected {1, 2, 3};
  EXPECT(expected == ABC.keys());
}

/* ************************************************************************* */
// Test scalar multiplication with * operator.
TEST(Expression, ScalarMultiply) {
  const Key key(67);
  const Point3_ expr = 23 * Point3_(key);

  const set<Key> expected_keys{key};
  EXPECT(expected_keys == expr.keys());

  map<Key, int> actual_dims, expected_dims {{key, 3}};
  expr.dims(actual_dims);
  EXPECT(actual_dims == expected_dims);

  // Check dims as map
  std::map<Key, int> map;
  expr.dims(map);
  LONGS_EQUAL(1, map.size());

  Values values;
  values.insert<Point3>(key, Point3(1, 0, 2));

  // Check value
  const Point3 expected(23, 0, 46);
  EXPECT(assert_equal(expected, expr.value(values)));

  // Check value + Jacobians
  std::vector<Matrix> H(1);
  EXPECT(assert_equal(expected, expr.value(values, H)));
  EXPECT(assert_equal(23 * I_3x3, H[0]));
}

/* ************************************************************************* */
// Test sum with + operator.
TEST(Expression, BinarySum) {
  const Key key(67);
  const Point3_ sum_ = Point3_(key) + Point3_(Point3(1, 1, 1));

  const set<Key> expected_keys{key};
  EXPECT(expected_keys == sum_.keys());

  map<Key, int> actual_dims, expected_dims {{key, 3}};
  sum_.dims(actual_dims);
  EXPECT(actual_dims == expected_dims);

  // Check dims as map
  std::map<Key, int> map;
  sum_.dims(map);
  LONGS_EQUAL(1, map.size());

  Values values;
  values.insert<Point3>(key, Point3(2, 2, 2));

  // Check value
  const Point3 expected(3, 3, 3);
  EXPECT(assert_equal(expected, sum_.value(values)));

  // Check value + Jacobians
  std::vector<Matrix> H(1);
  EXPECT(assert_equal(expected, sum_.value(values, H)));
  EXPECT(assert_equal(I_3x3, H[0]));
}

/* ************************************************************************* */
// Test sum of 3 variables with + operator.
TEST(Expression, TripleSum) {
  const Key key(67);
  const Point3_ p1_(Point3(1, 1, 1)), p2_(key);
  const Expression<Point3> sum_ = p1_ + p2_ + p1_;

  LONGS_EQUAL(1, sum_.keys().size());

  Values values;
  values.insert<Point3>(key, Point3(2, 2, 2));

  // Check value
  const Point3 expected(4, 4, 4);
  EXPECT(assert_equal(expected, sum_.value(values)));

  // Check value + Jacobians
  std::vector<Matrix> H(1);
  EXPECT(assert_equal(expected, sum_.value(values, H)));
  EXPECT(assert_equal(I_3x3, H[0]));
}

/* ************************************************************************* */
// Test sum with += operator.
TEST(Expression, PlusEqual) {
  const Key key(67);
  const Point3_ p1_(Point3(1, 1, 1)), p2_(key);
  Expression<Point3> sum_ = p1_;
  sum_ += p2_;
  sum_ += p1_;

  LONGS_EQUAL(1, sum_.keys().size());

  Values values;
  values.insert<Point3>(key, Point3(2, 2, 2));

  // Check value
  const Point3 expected(4, 4, 4);
  EXPECT(assert_equal(expected, sum_.value(values)));

  // Check value + Jacobians
  std::vector<Matrix> H(1);
  EXPECT(assert_equal(expected, sum_.value(values, H)));
  EXPECT(assert_equal(I_3x3, H[0]));
}

/* ************************************************************************* */
TEST(Expression, SumOfUnaries) {
  const Key key(67);
  const Double_ norm_(&gtsam::norm3, Point3_(key));
  const Double_ sum_ = norm_ + norm_;

  Values values;
  values.insert<Point3>(key, Point3(6, 0, 0));

  // Check value
  EXPECT_DOUBLES_EQUAL(12, sum_.value(values), 1e-9);

  // Check value + Jacobians
  std::vector<Matrix> H(1);
  EXPECT_DOUBLES_EQUAL(12, sum_.value(values, H), 1e-9);
  EXPECT(assert_equal(Vector3(2, 0, 0).transpose(), H[0]));
}

/* ************************************************************************* */
TEST(Expression, UnaryOfSum) {
  const Key key1(42), key2(67);
  const Point3_ sum_ = Point3_(key1) + Point3_(key2);
  const Double_ norm_(&gtsam::norm3, sum_);

  map<Key, int> actual_dims, expected_dims = {{key1, 3}, {key2, 3}};
  norm_.dims(actual_dims);
  EXPECT(actual_dims == expected_dims);

  Values values;
  values.insert<Point3>(key1, Point3(1, 0, 0));
  values.insert<Point3>(key2, Point3(0, 1, 0));

  // Check value
  EXPECT_DOUBLES_EQUAL(sqrt(2), norm_.value(values), 1e-9);

  // Check value + Jacobians
  std::vector<Matrix> H(2);
  EXPECT_DOUBLES_EQUAL(sqrt(2), norm_.value(values, H), 1e-9);
  EXPECT(assert_equal(0.5 * sqrt(2) * Vector3(1, 1, 0).transpose(), H[0]));
  EXPECT(assert_equal(0.5 * sqrt(2) * Vector3(1, 1, 0).transpose(), H[1]));
}

/* ************************************************************************* */
TEST(Expression, WeightedSum) {
  const Key key1(42), key2(67);
  const Point3_ weighted_sum_ = 17 * Point3_(key1) + 23 * Point3_(key2);

  map<Key, int> actual_dims, expected_dims {{key1, 3}, {key2, 3}};
  weighted_sum_.dims(actual_dims);
  EXPECT(actual_dims == expected_dims);

  Values values;
  const Point3 point1(1, 0, 0), point2(0, 1, 0);
  values.insert<Point3>(key1, point1);
  values.insert<Point3>(key2, point2);

  // Check value
  const Point3 expected = 17 * point1 + 23 * point2;
  EXPECT(assert_equal(expected, weighted_sum_.value(values)));

  // Check value + Jacobians
  std::vector<Matrix> H(2);
  EXPECT(assert_equal(expected, weighted_sum_.value(values, H)));
  EXPECT(assert_equal(17 * I_3x3, H[0]));
  EXPECT(assert_equal(23 * I_3x3, H[1]));
}

/* ************************************************************************* */
TEST(Expression, Subtract) {
  const Vector3 p = Vector3::Random(), q = Vector3::Random();
  Values values;
  values.insert(0, p);
  values.insert(1, q);
  const Vector3_ expression = Vector3_(0) - Vector3_(1);
  set<Key> expected_keys = {0, 1};
  EXPECT(expression.keys() == expected_keys);

  // Check value + Jacobians
  std::vector<Matrix> H(2);
  EXPECT(assert_equal<Vector3>(p - q, expression.value(values, H)));
  EXPECT(assert_equal(I_3x3, H[0]));
  EXPECT(assert_equal(-I_3x3, H[1]));
}

/* ************************************************************************* */
TEST(Expression, LinearExpression) {
  const Key key(67);
  const std::function<Vector3(Point3)> f = [](const Point3& p) { return (Vector3)p; };
  const Matrix3 kIdentity = I_3x3;
  const Expression<Vector3> linear_ = linearExpression(f, Point3_(key), kIdentity);

  Values values;
  values.insert<Point3>(key, Point3(1, 0, 2));

  // Check value
  const Vector3 expected(1, 0, 2);
  EXPECT(assert_equal(expected, linear_.value(values)));

  // Check value + Jacobians
  std::vector<Matrix> H(1);
  EXPECT(assert_equal(expected, linear_.value(values, H)));
  EXPECT(assert_equal(I_3x3, H[0]));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
