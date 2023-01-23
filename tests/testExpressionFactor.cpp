/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testExpressionFactor.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/ExpressionFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/expressions.h>

using namespace std::placeholders;

using namespace std;
using namespace gtsam;

Point2 measured(-17, 30);
SharedNoiseModel model = noiseModel::Unit::Create(2);

// This deals with the overload problem and makes the expressions factor
// understand that we work on Point3
Point2 (*Project)(const Point3&, OptionalJacobian<2, 3>) = &PinholeBase::Project;

namespace leaf {
// Create some values
struct MyValues: public Values {
  MyValues() {
    insert(2, Point2(3, 5));
  }
} values;

// Create leaf
Point2_ p(2);
}

/* ************************************************************************* */
// Leaf
TEST(ExpressionFactor, Leaf) {
  using namespace leaf;

  // Create old-style factor to create expected value and derivatives.
  PriorFactor<Point2> old(2, Point2(0, 0), model);

  // Create the equivalent factor with expression.
  ExpressionFactor<Point2> f(model, Point2(0, 0), p);

  // Check values and derivatives.
  EXPECT_DOUBLES_EQUAL(old.error(values), f.error(values), 1e-9);
  EXPECT_LONGS_EQUAL(2, f.dim());
  boost::shared_ptr<GaussianFactor> gf2 = f.linearize(values);
  EXPECT(assert_equal(*old.linearize(values), *gf2, 1e-9));
}

/* ************************************************************************* */
// Test leaf expression with noise model of different variance.
TEST(ExpressionFactor, Model) {
  using namespace leaf;

  SharedNoiseModel model = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.01));

  // Create old-style factor to create expected value and derivatives.
  PriorFactor<Point2> old(2, Point2(0, 0), model);

  // Create the equivalent factor with expression.
  ExpressionFactor<Point2> f(model, Point2(0, 0), p);

  // Check values and derivatives.
  EXPECT_DOUBLES_EQUAL(old.error(values), f.error(values), 1e-9);
  EXPECT_LONGS_EQUAL(2, f.dim());
  boost::shared_ptr<GaussianFactor> gf2 = f.linearize(values);
  EXPECT(assert_equal(*old.linearize(values), *gf2, 1e-9));
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, values, 1e-5, 1e-5); // another way
}

/* ************************************************************************* */
// Test leaf expression with constrained noise model.
TEST(ExpressionFactor, Constrained) {
  using namespace leaf;

  SharedDiagonal model = noiseModel::Constrained::MixedSigmas(Vector2(0.2, 0));

  // Create old-style factor to create expected value and derivatives
  PriorFactor<Point2> old(2, Point2(0, 0), model);

  // Concise version
  ExpressionFactor<Point2> f(model, Point2(0, 0), p);
  EXPECT_DOUBLES_EQUAL(old.error(values), f.error(values), 1e-9);
  EXPECT_LONGS_EQUAL(2, f.dim());
  boost::shared_ptr<GaussianFactor> gf2 = f.linearize(values);
  EXPECT(assert_equal(*old.linearize(values), *gf2, 1e-9));
}

/* ************************************************************************* */
// Unary(Leaf))
TEST(ExpressionFactor, Unary) {

  // Create some values
  Values values;
  values.insert(2, Point3(0, 0, 1));

  JacobianFactor expected( //
      2, (Matrix(2, 3) << 1, 0, 0, 0, 1, 0).finished(), //
      Vector2(-17, 30));

  // Create leaves
  Point3_ p(2);

  // Concise version
  ExpressionFactor<Point2> f(model, measured, project(p));
  EXPECT_LONGS_EQUAL(2, f.dim());
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  boost::shared_ptr<JacobianFactor> jf = //
      boost::dynamic_pointer_cast<JacobianFactor>(gf);
  EXPECT(assert_equal(expected, *jf, 1e-9));
}

/* ************************************************************************* */
// Unary(Leaf)) and Unary(Unary(Leaf)))
// wide version (not handled in fixed-size pipeline)
typedef Eigen::Matrix<double,9,3> Matrix93;
Vector9 wide(const Point3& p, OptionalJacobian<9,3> H) {
  Vector9 v;
  v << p, p, p;
  if (H) *H << I_3x3, I_3x3, I_3x3;
  return v;
}

typedef Eigen::Matrix<double,9,9> Matrix9;
Vector9 id9(const Vector9& v, OptionalJacobian<9,9> H) {
  if (H) *H = Matrix9::Identity();
  return v;
}

TEST(ExpressionFactor, Wide) {
  // Create some values
  Values values;
  values.insert(2, Point3(0, 0, 1));
  Point3_ point(2);
  Vector9 measured;
  measured.setZero();
  Expression<Vector9> expression(wide,point);
  SharedNoiseModel model = noiseModel::Unit::Create(9);

  ExpressionFactor<Vector9> f1(model, measured, expression);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f1, values, 1e-5, 1e-9);

  Expression<Vector9> expression2(id9,expression);
  ExpressionFactor<Vector9> f2(model, measured, expression2);
  EXPECT_CORRECT_FACTOR_JACOBIANS(f2, values, 1e-5, 1e-9);
}

/* ************************************************************************* */
static Point2 myUncal(const Cal3_S2& K, const Point2& p,
    OptionalJacobian<2,5> Dcal, OptionalJacobian<2,2> Dp) {
  return K.uncalibrate(p, Dcal, Dp);
}

// Binary(Leaf,Leaf)
TEST(ExpressionFactor, Binary) {

  typedef internal::BinaryExpression<Point2, Cal3_S2, Point2> Binary;

  Cal3_S2_ K_(1);
  Point2_ p_(2);
  Binary binary(myUncal, K_, p_);

  // Create some values
  Values values;
  values.insert(1, Cal3_S2());
  values.insert(2, Point2(0, 0));

  // Check size
  size_t size = binary.traceSize();
  // Use Variable Length Array, allocated on stack by gcc
  // Note unclear for Clang: http://clang.llvm.org/compatibility.html#vla
  internal::ExecutionTraceStorage traceStorage[size];
  internal::ExecutionTrace<Point2> trace;
  Point2 value = binary.traceExecution(values, trace, traceStorage);
  EXPECT(assert_equal(Point2(0,0),value, 1e-9));
  // trace.print();

  // Expected Jacobians
  Matrix25 expected25;
  expected25 << 0, 0, 0, 1, 0, 0, 0, 0, 0, 1;
  Matrix2 expected22;
  expected22 << 1, 0, 0, 1;

  // Check matrices
  std::optional<Binary::Record*> r = trace.record<Binary::Record>();
  CHECK(r);
  EXPECT(assert_equal(expected25, (Matrix ) (*r)->dTdA1, 1e-9));
  EXPECT(assert_equal(expected22, (Matrix ) (*r)->dTdA2, 1e-9));
}

/* ************************************************************************* */
// Unary(Binary(Leaf,Leaf))
TEST(ExpressionFactor, Shallow) {

  // Create some values
  Values values;
  values.insert(1, Pose3());
  values.insert(2, Point3(0, 0, 1));

  // Create old-style factor to create expected value and derivatives
  GenericProjectionFactor<Pose3, Point3> old(measured, model, 1, 2,
      boost::make_shared<Cal3_S2>());
  double expected_error = old.error(values);
  GaussianFactor::shared_ptr expected = old.linearize(values);

  // Create leaves
  Pose3_ x_(1);
  Point3_ p_(2);

  // Construct expression, concise version
  Point2_ expression = project(transformTo(x_, p_));

  // Get and check keys and dims
  KeyVector keys;
  FastVector<int> dims;
  boost::tie(keys, dims) = expression.keysAndDims();
  LONGS_EQUAL(2,keys.size());
  LONGS_EQUAL(2,dims.size());
  LONGS_EQUAL(1,keys[0]);
  LONGS_EQUAL(2,keys[1]);
  LONGS_EQUAL(6,dims[0]);
  LONGS_EQUAL(3,dims[1]);

  // traceExecution of shallow tree
  typedef internal::UnaryExpression<Point2, Point3> Unary;
  size_t size = expression.traceSize();
  internal::ExecutionTraceStorage traceStorage[size];
  internal::ExecutionTrace<Point2> trace;
  Point2 value = expression.traceExecution(values, trace, traceStorage);
  EXPECT(assert_equal(Point2(0,0),value, 1e-9));
  // trace.print();

  // Expected Jacobians
  Matrix23 expected23;
  expected23 << 1, 0, 0, 0, 1, 0;

  // Check matrices
  std::optional<Unary::Record*> r = trace.record<Unary::Record>();
  CHECK(r);
  EXPECT(assert_equal(expected23, (Matrix)(*r)->dTdA1, 1e-9));

  // Linearization
  ExpressionFactor<Point2> f2(model, measured, expression);
  EXPECT_DOUBLES_EQUAL(expected_error, f2.error(values), 1e-9);
  EXPECT_LONGS_EQUAL(2, f2.dim());
  boost::shared_ptr<GaussianFactor> gf2 = f2.linearize(values);
  EXPECT(assert_equal(*expected, *gf2, 1e-9));
}

/* ************************************************************************* */
// Binary(Leaf,Unary(Binary(Leaf,Leaf)))
TEST(ExpressionFactor, tree) {

  // Create some values
  Values values;
  values.insert(1, Pose3());
  values.insert(2, Point3(0, 0, 1));
  values.insert(3, Cal3_S2());

  // Create old-style factor to create expected value and derivatives
  GeneralSFMFactor2<Cal3_S2> old(measured, model, 1, 2, 3);
  double expected_error = old.error(values);
  GaussianFactor::shared_ptr expected = old.linearize(values);

  // Create leaves
  Pose3_ x(1);
  Point3_ p(2);
  Cal3_S2_ K(3);

  // Create expression tree
  Point3_ p_cam(x, &Pose3::transformTo, p);
  Point2_ xy_hat(Project, p_cam);
  Point2_ uv_hat(K, &Cal3_S2::uncalibrate, xy_hat);

  // Create factor and check value, dimension, linearization
  ExpressionFactor<Point2> f(model, measured, uv_hat);
  EXPECT_DOUBLES_EQUAL(expected_error, f.error(values), 1e-9);
  EXPECT_LONGS_EQUAL(2, f.dim());
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  EXPECT(assert_equal(*expected, *gf, 1e-9));

  // Concise version
  ExpressionFactor<Point2> f2(model, measured,
      uncalibrate(K, project(transformTo(x, p))));
  EXPECT_DOUBLES_EQUAL(expected_error, f2.error(values), 1e-9);
  EXPECT_LONGS_EQUAL(2, f2.dim());
  boost::shared_ptr<GaussianFactor> gf2 = f2.linearize(values);
  EXPECT(assert_equal(*expected, *gf2, 1e-9));

  // Try ternary version
  ExpressionFactor<Point2> f3(model, measured, project3(x, p, K));
  EXPECT_DOUBLES_EQUAL(expected_error, f3.error(values), 1e-9);
  EXPECT_LONGS_EQUAL(2, f3.dim());
  boost::shared_ptr<GaussianFactor> gf3 = f3.linearize(values);
  EXPECT(assert_equal(*expected, *gf3, 1e-9));
}

/* ************************************************************************* */
TEST(ExpressionFactor, Compose1) {

  // Create expression
  Rot3_ R1(1), R2(2);
  Rot3_ R3 = R1 * R2;

  // Create factor
  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), R3);

  // Create some values
  Values values;
  values.insert(1, Rot3());
  values.insert(2, Rot3());

  // Check unwhitenedError
  std::vector<Matrix> H(2);
  Vector actual = f.unwhitenedError(values, H);
  EXPECT(assert_equal(I_3x3, H[0],1e-9));
  EXPECT(assert_equal(I_3x3, H[1],1e-9));

  // Check linearization
  JacobianFactor expected(1, I_3x3, 2, I_3x3, Z_3x1);
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  boost::shared_ptr<JacobianFactor> jf = //
      boost::dynamic_pointer_cast<JacobianFactor>(gf);
  EXPECT(assert_equal(expected, *jf,1e-9));
}

/* ************************************************************************* */
// Test compose with arguments referring to the same rotation
TEST(ExpressionFactor, compose2) {

  // Create expression
  Rot3_ R1(1), R2(1);
  Rot3_ R3 = R1 * R2;

  // Create factor
  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), R3);

  // Create some values
  Values values;
  values.insert(1, Rot3());

  // Check unwhitenedError
  std::vector<Matrix> H(1);
  Vector actual = f.unwhitenedError(values, H);
  EXPECT_LONGS_EQUAL(1, H.size());
  EXPECT(assert_equal(2*I_3x3, H[0],1e-9));

  // Check linearization
  JacobianFactor expected(1, 2 * I_3x3, Z_3x1);
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  boost::shared_ptr<JacobianFactor> jf = //
      boost::dynamic_pointer_cast<JacobianFactor>(gf);
  EXPECT(assert_equal(expected, *jf,1e-9));
}

/* ************************************************************************* */
// Test compose with one arguments referring to a constant same rotation
TEST(ExpressionFactor, compose3) {

  // Create expression
  Rot3_ R1(Rot3::Identity()), R2(3);
  Rot3_ R3 = R1 * R2;

  // Create factor
  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), R3);

  // Create some values
  Values values;
  values.insert(3, Rot3());

  // Check unwhitenedError
  std::vector<Matrix> H(1);
  Vector actual = f.unwhitenedError(values, H);
  EXPECT_LONGS_EQUAL(1, H.size());
  EXPECT(assert_equal(I_3x3, H[0],1e-9));

  // Check linearization
  JacobianFactor expected(3, I_3x3, Z_3x1);
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  boost::shared_ptr<JacobianFactor> jf = //
      boost::dynamic_pointer_cast<JacobianFactor>(gf);
  EXPECT(assert_equal(expected, *jf,1e-9));
}

/* ************************************************************************* */
// Test compose with three arguments
Rot3 composeThree(const Rot3& R1, const Rot3& R2, const Rot3& R3,
    OptionalJacobian<3, 3> H1, OptionalJacobian<3, 3> H2, OptionalJacobian<3, 3> H3) {
  // return dummy derivatives (not correct, but that's ok for testing here)
  if (H1)
    *H1 = I_3x3;
  if (H2)
    *H2 = I_3x3;
  if (H3)
    *H3 = I_3x3;
  return R1 * (R2 * R3);
}

TEST(ExpressionFactor, composeTernary) {

  // Create expression
  Rot3_ A(1), B(2), C(3);
  Rot3_ ABC(composeThree, A, B, C);

  // Create factor
  ExpressionFactor<Rot3> f(noiseModel::Unit::Create(3), Rot3(), ABC);

  // Create some values
  Values values;
  values.insert(1, Rot3());
  values.insert(2, Rot3());
  values.insert(3, Rot3());

  // Check unwhitenedError
  std::vector<Matrix> H(3);
  Vector actual = f.unwhitenedError(values, H);
  EXPECT_LONGS_EQUAL(3, H.size());
  EXPECT(assert_equal(I_3x3, H[0],1e-9));
  EXPECT(assert_equal(I_3x3, H[1],1e-9));
  EXPECT(assert_equal(I_3x3, H[2],1e-9));

  // Check linearization
  JacobianFactor expected(1, I_3x3, 2, I_3x3, 3, I_3x3, Z_3x1);
  boost::shared_ptr<GaussianFactor> gf = f.linearize(values);
  boost::shared_ptr<JacobianFactor> jf = //
      boost::dynamic_pointer_cast<JacobianFactor>(gf);
  EXPECT(assert_equal(expected, *jf,1e-9));
}

TEST(ExpressionFactor, tree_finite_differences) {

  // Create some values
  Values values;
  values.insert(1, Pose3());
  values.insert(2, Point3(0, 0, 1));
  values.insert(3, Cal3_S2());

  // Create leaves
  Pose3_ x(1);
  Point3_ p(2);
  Cal3_S2_ K(3);

  // Create expression tree
  Point3_ p_cam(x, &Pose3::transformTo, p);
  Point2_ xy_hat(Project, p_cam);
  Point2_ uv_hat(K, &Cal3_S2::uncalibrate, xy_hat);

  const double fd_step = 1e-5;
  const double tolerance = 1e-5;
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(uv_hat, values, fd_step, tolerance);
}

TEST(ExpressionFactor, push_back) {
  NonlinearFactorGraph graph;
  graph.addExpressionFactor(model, Point2(0, 0), leaf::p);
}

/* ************************************************************************* */
// Test with multiple compositions on duplicate keys
struct Combine {
  double a, b;
  Combine(double a, double b) : a(a), b(b) {}
  double operator()(const double& x, const double& y, OptionalJacobian<1, 1> H1,
                    OptionalJacobian<1, 1> H2) {
    if (H1) (*H1) << a;
    if (H2) (*H2) << b;
    return a * x + b * y;
  }
};

TEST(Expression, testMultipleCompositions) {
  const double tolerance = 1e-5;
  const double fd_step = 1e-5;

  Values values;
  values.insert(1, 10.0);
  values.insert(2, 20.0);

  Expression<double> v1_(Key(1));
  Expression<double> v2_(Key(2));

  // BinaryExpression(1,2)
  //   Leaf, key = 1
  //   Leaf, key = 2
  Expression<double> sum1_(Combine(1, 2), v1_, v2_);
  EXPECT((sum1_.keys() == std::set<Key>{1, 2}));
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(sum1_, values, fd_step, tolerance);

  // BinaryExpression(3,4)
  //   BinaryExpression(1,2)
  //     Leaf, key = 1
  //     Leaf, key = 2
  //   Leaf, key = 1
  Expression<double> sum2_(Combine(3, 4), sum1_, v1_);
  EXPECT((sum2_.keys() == std::set<Key>{1, 2}));
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(sum2_, values, fd_step, tolerance);

  // BinaryExpression(5,6)
  //   BinaryExpression(3,4)
  //     BinaryExpression(1,2)
  //       Leaf, key = 1
  //       Leaf, key = 2
  //     Leaf, key = 1
  //   BinaryExpression(1,2)
  //     Leaf, key = 1
  //     Leaf, key = 2
  Expression<double> sum3_(Combine(5, 6), sum1_, sum2_);
  EXPECT((sum3_.keys() == std::set<Key>{1, 2}));
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(sum3_, values, fd_step, tolerance);
}

/* ************************************************************************* */
// Another test, with Ternary Expressions
static double combine3(const double& x, const double& y, const double& z,
                        OptionalJacobian<1, 1> H1, OptionalJacobian<1, 1> H2,
                        OptionalJacobian<1, 1> H3) {
  if (H1) (*H1) << 1.0;
  if (H2) (*H2) << 2.0;
  if (H3) (*H3) << 3.0;
  return x + 2.0 * y + 3.0 * z;
}

TEST(Expression, testMultipleCompositions2) {
  const double tolerance = 1e-5;
  const double fd_step = 1e-5;

  Values values;
  values.insert(1, 10.0);
  values.insert(2, 20.0);
  values.insert(3, 30.0);

  Expression<double> v1_(Key(1));
  Expression<double> v2_(Key(2));
  Expression<double> v3_(Key(3));

  Expression<double> sum1_(Combine(4,5), v1_, v2_);
  EXPECT((sum1_.keys() == std::set<Key>{1, 2}));
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(sum1_, values, fd_step, tolerance);

  Expression<double> sum2_(combine3, v1_, v2_, v3_);
  EXPECT((sum2_.keys() == std::set<Key>{1, 2, 3}));
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(sum2_, values, fd_step, tolerance);

  Expression<double> sum3_(combine3, v3_, v2_, v1_);
  EXPECT((sum3_.keys() == std::set<Key>{1, 2, 3}));
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(sum3_, values, fd_step, tolerance);

  Expression<double> sum4_(combine3, sum1_, sum2_, sum3_);
  EXPECT((sum4_.keys() == std::set<Key>{1, 2, 3}));
  EXPECT_CORRECT_EXPRESSION_JACOBIANS(sum4_, values, fd_step, tolerance);
}

/* ************************************************************************* */
// Test multiplication with the inverse of a matrix
TEST(ExpressionFactor, MultiplyWithInverse) {
  auto model = noiseModel::Isotropic::Sigma(3, 1);

  // Create expression
  Vector3_ f_expr(MultiplyWithInverse<3>(), Expression<Matrix3>(0), Vector3_(1));

  // Check derivatives
  Values values;
  Matrix3 A = Vector3(1, 2, 3).asDiagonal();
  A(0, 1) = 0.1;
  A(0, 2) = 0.1;
  const Vector3 b(0.1, 0.2, 0.3);
  values.insert<Matrix3>(0, A);
  values.insert<Vector3>(1, b);
  ExpressionFactor<Vector3> factor(model, Vector3::Zero(), f_expr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

/* ************************************************************************* */
// Test multiplication with the inverse of a matrix function
namespace test_operator {
Vector3 f(const Point2& a, const Vector3& b, OptionalJacobian<3, 2> H1,
          OptionalJacobian<3, 3> H2) {
  Matrix3 A = Vector3(1, 2, 3).asDiagonal();
  A(0, 1) = a.x();
  A(0, 2) = a.y();
  A(1, 0) = a.x();
  if (H1) *H1 << b.y(), b.z(), b.x(), 0, 0, 0;
  if (H2) *H2 = A;
  return A * b;
}
}

TEST(ExpressionFactor, MultiplyWithInverseFunction) {
  auto model = noiseModel::Isotropic::Sigma(3, 1);

  using test_operator::f;
  Vector3_ f_expr(MultiplyWithInverseFunction<Point2, 3>(f),
                  Expression<Point2>(0), Vector3_(1));

  // Check derivatives
  Point2 a(1, 2);
  const Vector3 b(0.1, 0.2, 0.3);
  Matrix32 H1;
  Matrix3 A;
  const Vector Ab = f(a, b, H1, A);
  CHECK(assert_equal(A * b, Ab));
  CHECK(assert_equal(
      numericalDerivative11<Vector3, Point2>(
        [&](const Point2& a) { return f(a, b, {}, {}); }, a),
      H1));

  Values values;
  values.insert<Point2>(0, a);
  values.insert<Vector3>(1, b);
  ExpressionFactor<Vector3> factor(model, Vector3::Zero(), f_expr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}


/* ************************************************************************* */
// Test N-ary variadic template
class TestNaryFactor
    : public gtsam::ExpressionFactorN<gtsam::Point3 /*return type*/,
                                      gtsam::Rot3, gtsam::Point3, 
                                      gtsam::Rot3, gtsam::Point3> {
private:
  using This = TestNaryFactor;
  using Base =
      gtsam::ExpressionFactorN<gtsam::Point3 /*return type*/,
        gtsam::Rot3, gtsam::Point3, gtsam::Rot3, gtsam::Point3>;

public:
  /// default constructor
  TestNaryFactor() = default;
  ~TestNaryFactor() override = default;

  TestNaryFactor(gtsam::Key kR1, gtsam::Key kV1,  gtsam::Key kR2, gtsam::Key kV2,
    const gtsam::SharedNoiseModel &model, const gtsam::Point3& measured)
      : Base({kR1, kV1, kR2, kV2}, model, measured) {
    this->initialize(expression({kR1, kV1, kR2, kV2}));
  }

  /// @return a deep copy of this factor
  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  // Return measurement expression
  gtsam::Expression<gtsam::Point3> expression(
      const std::array<gtsam::Key, NARY_EXPRESSION_SIZE> &keys) const override {
    gtsam::Expression<gtsam::Rot3>   R1_(keys[0]);
    gtsam::Expression<gtsam::Point3> V1_(keys[1]);
    gtsam::Expression<gtsam::Rot3>   R2_(keys[2]);
    gtsam::Expression<gtsam::Point3> V2_(keys[3]);
    return {gtsam::rotate(R1_, V1_) - gtsam::rotate(R2_, V2_)};
  }

  /** print */
  void print(const std::string &s,
             const gtsam::KeyFormatter &keyFormatter =
                 gtsam::DefaultKeyFormatter) const override {
    std::cout << s << "TestNaryFactor("
              << keyFormatter(Factor::keys_[0]) << ","
              << keyFormatter(Factor::keys_[1]) << ","
              << keyFormatter(Factor::keys_[2]) << ","
              << keyFormatter(Factor::keys_[3]) << ")\n";
    gtsam::traits<gtsam::Point3>::Print(measured_, "  measured: ");
    this->noiseModel_->print("  noise model: ");
  }

  /** equals */
  bool equals(const gtsam::NonlinearFactor &expected,
              double tol = 1e-9) const override {
    const This *e = dynamic_cast<const This *>(&expected);
    return e != nullptr && Base::equals(*e, tol) && 
      gtsam::traits<gtsam::Point3>::Equals(measured_,e->measured_, tol);
  }

private:
  /** Serialization function */
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE &ar, const unsigned int /*version*/) {
    ar &boost::serialization::make_nvp(
        "TestNaryFactor",
        boost::serialization::base_object<Base>(*this));
    ar &BOOST_SERIALIZATION_NVP(measured_);
  }
};

TEST(ExpressionFactor, variadicTemplate) {
  using gtsam::symbol_shorthand::R;
  using gtsam::symbol_shorthand::V;

  // Create factor
  TestNaryFactor f(R(0),V(0), R(1), V(1), noiseModel::Unit::Create(3), Point3(0,0,0));
  
  // Create some values
  Values values;
  values.insert(R(0), Rot3::Ypr(0.1, 0.2, 0.3));
  values.insert(V(0), Point3(1, 2, 3));
  values.insert(R(1), Rot3::Ypr(0.2, 0.5, 0.2));
  values.insert(V(1), Point3(5, 6, 7));

  // Check unwhitenedError
  std::vector<Matrix> H(4);
  Vector actual = f.unwhitenedError(values, H);
  EXPECT_LONGS_EQUAL(4, H.size());
  EXPECT(assert_equal(Eigen::Vector3d(-5.63578115, -4.85353243, -1.4801204), actual, 1e-5));
  
  EXPECT_CORRECT_FACTOR_JACOBIANS(f, values, 1e-8, 1e-5);
}

TEST(ExpressionFactor, normalize) {
  auto model = noiseModel::Isotropic::Sigma(3, 1);

  // Create expression
  const auto x = Vector3_(1);
  Vector3_ f_expr = normalize(x);

  // Check derivatives
  Values values;
  values.insert(1, Vector3(1, 2, 3));
  ExpressionFactor<Vector3> factor(model, Vector3(1.0/sqrt(14), 2.0/sqrt(14), 3.0/sqrt(14)), f_expr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

TEST(ExpressionFactor, crossProduct) {
  auto model = noiseModel::Isotropic::Sigma(3, 1);

  // Create expression
  const auto a = Vector3_(1);
  const auto b = Vector3_(2);
  Vector3_ f_expr = cross(a, b);

  // Check derivatives
  Values values;
  values.insert(1, Vector3(0.1, 0.2, 0.3));
  values.insert(2, Vector3(0.4, 0.5, 0.6));
  ExpressionFactor<Vector3> factor(model, Vector3::Zero(), f_expr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}

TEST(ExpressionFactor, dotProduct) {
  auto model = noiseModel::Isotropic::Sigma(1, 1);

  // Create expression
  const auto a = Vector3_(1);
  const auto b = Vector3_(2);
  Double_ f_expr = dot(a, b);

  // Check derivatives
  Values values;
  values.insert(1, Vector3(0.1, 0.2, 0.3));
  values.insert(2, Vector3(0.4, 0.5, 0.6));
  ExpressionFactor<double> factor(model, .0, f_expr);
  EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-5, 1e-5);
}


/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

