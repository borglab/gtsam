/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  testNonlinearFactor.cpp
 *  @brief Unit tests for Non-Linear Factor,
 *  create a non linear factor graph and a values structure for it and
 *  calculate the error for the factor.
 *  @author Christian Potthast
 **/

/*STL/C++*/
#include <iostream>

#include <CppUnitLite/TestHarness.h>

// TODO: DANGEROUS, create shared pointers
#define GTSAM_MAGIC_GAUSSIAN 2

#include <gtsam/base/Testable.h>
#include <gtsam/base/Matrix.h>
#include <tests/smallExample.h>
#include <tests/simulated2D.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

using namespace std;
using namespace gtsam;
using namespace example;

// Convenience for named keys
using symbol_shorthand::X;
using symbol_shorthand::L;

typedef boost::shared_ptr<NonlinearFactor > shared_nlf;

/* ************************************************************************* */
TEST( NonlinearFactor, equals )
{
  SharedNoiseModel sigma(noiseModel::Isotropic::Sigma(2,1.0));

  // create two nonlinear2 factors
  Point2 z3(0.,-1.);
  simulated2D::Measurement f0(z3, sigma, X(1),L(1));

  // measurement between x2 and l1
  Point2 z4(-1.5, -1.);
  simulated2D::Measurement f1(z4, sigma, X(2),L(1));

  CHECK(assert_equal(f0,f0));
  CHECK(f0.equals(f0));
  CHECK(!f0.equals(f1));
  CHECK(!f1.equals(f0));
}

/* ************************************************************************* */
TEST( NonlinearFactor, equals2 )
{
  // create a non linear factor graph
  NonlinearFactorGraph fg = createNonlinearFactorGraph();

  // get two factors
  NonlinearFactorGraph::sharedFactor f0 = fg[0], f1 = fg[1];

  CHECK(f0->equals(*f0));
  CHECK(!f0->equals(*f1));
  CHECK(!f1->equals(*f0));
}

/* ************************************************************************* */
TEST( NonlinearFactor, NonlinearFactor )
{
  // create a non linear factor graph
  NonlinearFactorGraph fg = createNonlinearFactorGraph();

  // create a values structure for the non linear factor graph
  Values cfg = createNoisyValues();

  // get the factor "f1" from the factor graph
  NonlinearFactorGraph::sharedFactor factor = fg[0];

  // calculate the error_vector from the factor "f1"
  // error_vector = [0.1 0.1]
  Vector actual_e = boost::dynamic_pointer_cast<NoiseModelFactor>(factor)->unwhitenedError(cfg);
  CHECK(assert_equal(0.1*Vector::Ones(2),actual_e));

  // error = 0.5 * [1 1] * [1;1] = 1
  double expected = 1.0;

  // calculate the error from the factor "f1"
  double actual = factor->error(cfg);
  DOUBLES_EQUAL(expected,actual,0.00000001);
}

/* ************************************************************************* */
TEST(NonlinearFactor, Weight) {
  // create a values structure for the non linear factor graph
  Values values;

  // Instantiate a concrete class version of a NoiseModelFactor
  PriorFactor<Point2> factor1(X(1), Point2(0, 0));
  values.insert(X(1), Point2(0.1, 0.1));

  CHECK(assert_equal(1.0, factor1.weight(values)));

  // Factor with noise model
  auto noise = noiseModel::Isotropic::Sigma(2, 0.2);
  PriorFactor<Point2> factor2(X(2), Point2(1, 1), noise);
  values.insert(X(2), Point2(1.1, 1.1));

  CHECK(assert_equal(1.0, factor2.weight(values)));

  Point2 estimate(3, 3), prior(1, 1);
  double distance = (estimate - prior).norm();

  auto gaussian = noiseModel::Isotropic::Sigma(2, 0.2);

  PriorFactor<Point2> factor;

  // vector to store all the robust models in so we can test iteratively.
  vector<noiseModel::Robust::shared_ptr> robust_models;

  // Fair noise model
  auto fair = noiseModel::Robust::Create(
      noiseModel::mEstimator::Fair::Create(1.3998), gaussian);
  robust_models.push_back(fair);

  // Huber noise model
  auto huber = noiseModel::Robust::Create(
      noiseModel::mEstimator::Huber::Create(1.345), gaussian);
  robust_models.push_back(huber);

  // Cauchy noise model
  auto cauchy = noiseModel::Robust::Create(
      noiseModel::mEstimator::Cauchy::Create(0.1), gaussian);
  robust_models.push_back(cauchy);

  // Tukey noise model
  auto tukey = noiseModel::Robust::Create(
      noiseModel::mEstimator::Tukey::Create(4.6851), gaussian);
  robust_models.push_back(tukey);

  // Welsch noise model
  auto welsch = noiseModel::Robust::Create(
      noiseModel::mEstimator::Welsch::Create(2.9846), gaussian);
  robust_models.push_back(welsch);

  // Geman-McClure noise model
  auto gm = noiseModel::Robust::Create(
      noiseModel::mEstimator::GemanMcClure::Create(1.0), gaussian);
  robust_models.push_back(gm);

  // DCS noise model
  auto dcs = noiseModel::Robust::Create(
      noiseModel::mEstimator::DCS::Create(1.0), gaussian);
  robust_models.push_back(dcs);

  // L2WithDeadZone noise model
  auto l2 = noiseModel::Robust::Create(
      noiseModel::mEstimator::L2WithDeadZone::Create(1.0), gaussian);
  robust_models.push_back(l2);

  for(auto&& model: robust_models) {
    factor = PriorFactor<Point2>(X(3), prior, model);
    values.clear();
    values.insert(X(3), estimate);
    CHECK(assert_equal(model->robust()->weight(distance), factor.weight(values)));
  }
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f1 )
{
  Values c = createNoisyValues();

  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  NonlinearFactorGraph::sharedFactor nlf = nfg[0];

  // We linearize at noisy config from SmallExample
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[0];

  CHECK(assert_equal(*expected,*actual));

  // The error |A*dx-b| approximates (h(x0+dx)-z) = -error_vector
  // Hence i.e., b = approximates z-h(x0) = error_vector(x0)
  //CHECK(assert_equal(nlf->error_vector(c),actual->get_b()));
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f2 )
{
  Values c = createNoisyValues();

  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  NonlinearFactorGraph::sharedFactor nlf = nfg[1];

  // We linearize at noisy config from SmallExample
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[1];

  CHECK(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f3 )
{
  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  NonlinearFactorGraph::sharedFactor nlf = nfg[2];

  // We linearize at noisy config from SmallExample
  Values c = createNoisyValues();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[2];

  CHECK(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_f4 )
{
  // Grab a non-linear factor
  NonlinearFactorGraph nfg = createNonlinearFactorGraph();
  NonlinearFactorGraph::sharedFactor nlf = nfg[3];

  // We linearize at noisy config from SmallExample
  Values c = createNoisyValues();
  GaussianFactor::shared_ptr actual = nlf->linearize(c);

  GaussianFactorGraph lfg = createGaussianFactorGraph();
  GaussianFactor::shared_ptr expected = lfg[3];

  CHECK(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, size )
{
  // create a non linear factor graph
  NonlinearFactorGraph fg = createNonlinearFactorGraph();

  // create a values structure for the non linear factor graph
  Values cfg = createNoisyValues();

  // get some factors from the graph
  NonlinearFactorGraph::sharedFactor factor1 = fg[0], factor2 = fg[1],
      factor3 = fg[2];

  CHECK(factor1->size() == 1);
  CHECK(factor2->size() == 2);
  CHECK(factor3->size() == 2);
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_constraint1 )
{
  SharedDiagonal constraint = noiseModel::Constrained::MixedSigmas(Vector2(0.2,0));

  Point2 mu(1., -1.);
  NonlinearFactorGraph::sharedFactor f0(new simulated2D::Prior(mu, constraint, X(1)));

  Values config;
  config.insert(X(1), Point2(1.0, 2.0));
  GaussianFactor::shared_ptr actual = f0->linearize(config);

  // create expected
  Vector2 b(0., -3.);
  JacobianFactor expected(X(1), (Matrix(2, 2) << 5.0, 0.0, 0.0, 1.0).finished(), b,
    noiseModel::Constrained::MixedSigmas(Vector2(1,0)));
  CHECK(assert_equal((const GaussianFactor&)expected, *actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, linearize_constraint2 )
{
  SharedDiagonal constraint = noiseModel::Constrained::MixedSigmas(Vector2(0.2,0));

  Point2 z3(1.,-1.);
  simulated2D::Measurement f0(z3, constraint, X(1),L(1));

  Values config;
  config.insert(X(1), Point2(1.0, 2.0));
  config.insert(L(1), Point2(5.0, 4.0));
  GaussianFactor::shared_ptr actual = f0.linearize(config);

  // create expected
  Matrix2 A; A << 5.0, 0.0, 0.0, 1.0;
  Vector2 b(-15., -3.);
  JacobianFactor expected(X(1), -1*A, L(1), A, b,
    noiseModel::Constrained::MixedSigmas(Vector2(1,0)));
  CHECK(assert_equal((const GaussianFactor&)expected, *actual));
}

/* ************************************************************************* */
TEST( NonlinearFactor, cloneWithNewNoiseModel )
{
  // create original factor
  double sigma1 = 0.1;
  NonlinearFactorGraph nfg = example::nonlinearFactorGraphWithGivenSigma(sigma1);

  // create expected
  double sigma2 = 10;
  NonlinearFactorGraph expected = example::nonlinearFactorGraphWithGivenSigma(sigma2);

  // create actual
  NonlinearFactorGraph actual;
  SharedNoiseModel noise2 = noiseModel::Isotropic::Sigma(2,sigma2);
  actual.push_back( boost::dynamic_pointer_cast<NoiseModelFactor>(nfg[0])->cloneWithNewNoiseModel(noise2) );

  // check it's all good
  CHECK(assert_equal(expected, actual));
}

/* ************************************************************************* */
class TestFactor1 : public NoiseModelFactor1<double> {
  static_assert(std::is_same<Base, NoiseModelFactor>::value, "Base type wrong");
  static_assert(std::is_same<This, NoiseModelFactor1<double>>::value,
                "This type wrong");

 public:
  typedef NoiseModelFactor1<double> Base;
  using Base::evaluateError;
  TestFactor1() : Base(noiseModel::Diagonal::Sigmas(Vector1(2.0)), L(1)) {}
  using Base::NoiseModelFactor1;  // inherit constructors

  Vector evaluateError(const double& x1, OptionalMatrixType H1) const override {
    if (H1) *H1 = (Matrix(1, 1) << 1.0).finished();
    return (Vector(1) << x1).finished();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new TestFactor1(*this)));
  }
};

/* ************************************ */
TEST(NonlinearFactor, NoiseModelFactor1) {
  TestFactor1 tf;
  Values tv;
  tv.insert(L(1), double((1.0)));
  EXPECT(assert_equal((Vector(1) << 1.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(0.25 / 2.0, tf.error(tv), 1e-9);
  JacobianFactor jf(
      *boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv)));
  LONGS_EQUAL((long)L(1), (long)jf.keys()[0]);
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 0.5).finished(),
                      jf.getA(jf.begin())));
  EXPECT(assert_equal((Vector)(Vector(1) << -0.5).finished(), jf.getb()));

  // Test all functions/types for backwards compatibility
  static_assert(std::is_same<TestFactor1::X, double>::value,
                "X type incorrect");
  EXPECT(assert_equal(tf.key(), L(1)));
  std::vector<Matrix> H = {Matrix()};
  EXPECT(assert_equal(Vector1(1.0), tf.unwhitenedError(tv, H)));

  // Test constructors
  TestFactor1 tf2(noiseModel::Unit::Create(1), L(1));
  TestFactor1 tf3(noiseModel::Unit::Create(1), {L(1)});
  TestFactor1 tf4(noiseModel::Unit::Create(1), gtsam::Symbol('L', 1));
}

/* ************************************************************************* */
class TestFactor4 : public NoiseModelFactor4<double, double, double, double> {
  static_assert(std::is_same<Base, NoiseModelFactor>::value, "Base type wrong");
  static_assert(
      std::is_same<This,
                   NoiseModelFactor4<double, double, double, double>>::value,
      "This type wrong");

 public:
  typedef NoiseModelFactor4<double, double, double, double> Base;
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  TestFactor4() : Base(noiseModel::Diagonal::Sigmas((Vector(1) << 2.0).finished()), X(1), X(2), X(3), X(4)) {}
  using Base::NoiseModelFactor4;  // inherit constructors

  Vector
    evaluateError(const double& x1, const double& x2, const double& x3, const double& x4,
        OptionalMatrixType H1, OptionalMatrixType H2,
        OptionalMatrixType H3, OptionalMatrixType H4) const override {
    if(H1) {
      *H1 = (Matrix(1, 1) << 1.0).finished();
      *H2 = (Matrix(1, 1) << 2.0).finished();
      *H3 = (Matrix(1, 1) << 3.0).finished();
      *H4 = (Matrix(1, 1) << 4.0).finished();
    }
    return (Vector(1) << x1 + 2.0 * x2 + 3.0 * x3 + 4.0 * x4).finished();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new TestFactor4(*this))); }
};

/* ************************************ */
TEST(NonlinearFactor, NoiseModelFactor4) {
  TestFactor4 tf;
  Values tv;
  tv.insert(X(1), double((1.0)));
  tv.insert(X(2), double((2.0)));
  tv.insert(X(3), double((3.0)));
  tv.insert(X(4), double((4.0)));
  EXPECT(assert_equal((Vector(1) << 30.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(0.5 * 30.0 * 30.0 / 4.0, tf.error(tv), 1e-9);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv)));
  LONGS_EQUAL((long)X(1), (long)jf.keys()[0]);
  LONGS_EQUAL((long)X(2), (long)jf.keys()[1]);
  LONGS_EQUAL((long)X(3), (long)jf.keys()[2]);
  LONGS_EQUAL((long)X(4), (long)jf.keys()[3]);
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 0.5).finished(), jf.getA(jf.begin())));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.0).finished(), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.5).finished(), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.0).finished(), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal((Vector)(Vector(1) << 0.5 * -30.).finished(), jf.getb()));

  // Test all functions/types for backwards compatibility
  static_assert(std::is_same<TestFactor4::X1, double>::value,
                "X1 type incorrect");
  static_assert(std::is_same<TestFactor4::X2, double>::value,
                "X2 type incorrect");
  static_assert(std::is_same<TestFactor4::X3, double>::value,
                "X3 type incorrect");
  static_assert(std::is_same<TestFactor4::X4, double>::value,
                "X4 type incorrect");
  EXPECT(assert_equal(tf.key1(), X(1)));
  EXPECT(assert_equal(tf.key2(), X(2)));
  EXPECT(assert_equal(tf.key3(), X(3)));
  EXPECT(assert_equal(tf.key4(), X(4)));
  std::vector<Matrix> H = {Matrix(), Matrix(), Matrix(), Matrix()};
  EXPECT(assert_equal(Vector1(30.0), tf.unwhitenedError(tv, H)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.).finished(), H.at(0)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.).finished(), H.at(1)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 3.).finished(), H.at(2)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 4.).finished(), H.at(3)));

  // And test "forward compatibility" using `key<N>` and `ValueType<N>` too
  static_assert(std::is_same<TestFactor4::ValueType<1>, double>::value,
                "ValueType<1> type incorrect");
  static_assert(std::is_same<TestFactor4::ValueType<2>, double>::value,
                "ValueType<2> type incorrect");
  static_assert(std::is_same<TestFactor4::ValueType<3>, double>::value,
                "ValueType<3> type incorrect");
  static_assert(std::is_same<TestFactor4::ValueType<4>, double>::value,
                "ValueType<4> type incorrect");
  EXPECT(assert_equal(tf.key<1>(), X(1)));
  EXPECT(assert_equal(tf.key<2>(), X(2)));
  EXPECT(assert_equal(tf.key<3>(), X(3)));
  EXPECT(assert_equal(tf.key<4>(), X(4)));

  // Test constructors
  TestFactor4 tf2(noiseModel::Unit::Create(1), L(1), L(2), L(3), L(4));
  TestFactor4 tf3(noiseModel::Unit::Create(1), {L(1), L(2), L(3), L(4)});
  TestFactor4 tf4(noiseModel::Unit::Create(1),
                  std::array<Key, 4>{L(1), L(2), L(3), L(4)});
  std::vector<Key> keys = {L(1), L(2), L(3), L(4)};
  TestFactor4 tf5(noiseModel::Unit::Create(1), keys);
}

/* ************************************************************************* */
class TestFactor5 : public NoiseModelFactor5<double, double, double, double, double> {
public:
  typedef NoiseModelFactor5<double, double, double, double, double> Base;
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  TestFactor5() : Base(noiseModel::Diagonal::Sigmas((Vector(1) << 2.0).finished()), X(1), X(2), X(3), X(4), X(5)) {}

  Vector
    evaluateError(const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5,
        OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3, 
		OptionalMatrixType H4, OptionalMatrixType H5) const override {
    if(H1) {
      *H1 = (Matrix(1, 1) << 1.0).finished();
      *H2 = (Matrix(1, 1) << 2.0).finished();
      *H3 = (Matrix(1, 1) << 3.0).finished();
      *H4 = (Matrix(1, 1) << 4.0).finished();
      *H5 = (Matrix(1, 1) << 5.0).finished();
    }
    return (Vector(1) << x1 + 2.0 * x2 + 3.0 * x3 + 4.0 * x4 + 5.0 * x5)
        .finished();
  }
};

/* ************************************ */
TEST(NonlinearFactor, NoiseModelFactor5) {
  TestFactor5 tf;
  Values tv;
  tv.insert(X(1), double((1.0)));
  tv.insert(X(2), double((2.0)));
  tv.insert(X(3), double((3.0)));
  tv.insert(X(4), double((4.0)));
  tv.insert(X(5), double((5.0)));
  EXPECT(assert_equal((Vector(1) << 55.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(0.5 * 55.0 * 55.0 / 4.0, tf.error(tv), 1e-9);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv)));
  LONGS_EQUAL((long)X(1), (long)jf.keys()[0]);
  LONGS_EQUAL((long)X(2), (long)jf.keys()[1]);
  LONGS_EQUAL((long)X(3), (long)jf.keys()[2]);
  LONGS_EQUAL((long)X(4), (long)jf.keys()[3]);
  LONGS_EQUAL((long)X(5), (long)jf.keys()[4]);
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 0.5).finished(), jf.getA(jf.begin())));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.0).finished(), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.5).finished(), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.0).finished(), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.5).finished(), jf.getA(jf.begin()+4)));
  EXPECT(assert_equal((Vector)(Vector(1) << 0.5 * -55.).finished(), jf.getb()));
}

/* ************************************************************************* */
class TestFactor6 : public NoiseModelFactor6<double, double, double, double, double, double> {
public:
  typedef NoiseModelFactor6<double, double, double, double, double, double> Base;
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  TestFactor6() : Base(noiseModel::Diagonal::Sigmas((Vector(1) << 2.0).finished()), X(1), X(2), X(3), X(4), X(5), X(6)) {}

  Vector
    evaluateError(const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6,
        OptionalMatrixType H1, OptionalMatrixType H2, OptionalMatrixType H3, OptionalMatrixType H4, 
		OptionalMatrixType H5, OptionalMatrixType H6) const override {
    if(H1) {
      *H1 = (Matrix(1, 1) << 1.0).finished();
      *H2 = (Matrix(1, 1) << 2.0).finished();
      *H3 = (Matrix(1, 1) << 3.0).finished();
      *H4 = (Matrix(1, 1) << 4.0).finished();
      *H5 = (Matrix(1, 1) << 5.0).finished();
      *H6 = (Matrix(1, 1) << 6.0).finished();
    }
    return (Vector(1) << x1 + 2.0 * x2 + 3.0 * x3 + 4.0 * x4 + 5.0 * x5 +
                             6.0 * x6)
        .finished();
  }

};

/* ************************************ */
TEST(NonlinearFactor, NoiseModelFactor6) {
  TestFactor6 tf;
  Values tv;
  tv.insert(X(1), double((1.0)));
  tv.insert(X(2), double((2.0)));
  tv.insert(X(3), double((3.0)));
  tv.insert(X(4), double((4.0)));
  tv.insert(X(5), double((5.0)));
  tv.insert(X(6), double((6.0)));
  EXPECT(assert_equal((Vector(1) << 91.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(0.5 * 91.0 * 91.0 / 4.0, tf.error(tv), 1e-9);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv)));
  LONGS_EQUAL((long)X(1), (long)jf.keys()[0]);
  LONGS_EQUAL((long)X(2), (long)jf.keys()[1]);
  LONGS_EQUAL((long)X(3), (long)jf.keys()[2]);
  LONGS_EQUAL((long)X(4), (long)jf.keys()[3]);
  LONGS_EQUAL((long)X(5), (long)jf.keys()[4]);
  LONGS_EQUAL((long)X(6), (long)jf.keys()[5]);
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 0.5).finished(), jf.getA(jf.begin())));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.0).finished(), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.5).finished(), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.0).finished(), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.5).finished(), jf.getA(jf.begin()+4)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 3.0).finished(), jf.getA(jf.begin()+5)));
  EXPECT(assert_equal((Vector)(Vector(1) << 0.5 * -91.).finished(), jf.getb()));

}

/* ************************************************************************* */
class TestFactorN : public NoiseModelFactorN<double, double, double, double> {
public:
  typedef NoiseModelFactorN<double, double, double, double> Base;
  // Provide access to the Matrix& version of evaluateError:
  using Base::evaluateError;
  using Type1 = ValueType<1>;  // Test that we can use the ValueType<> template

  TestFactorN() : Base(noiseModel::Diagonal::Sigmas((Vector(1) << 2.0).finished()), X(1), X(2), X(3), X(4)) {}

  Vector
    evaluateError(const double& x1, const double& x2, const double& x3, const double& x4,
        OptionalMatrixType H1, OptionalMatrixType H2,
        OptionalMatrixType H3, OptionalMatrixType H4) const override {
    if (H1) *H1 = (Matrix(1, 1) << 1.0).finished();
    if (H2) *H2 = (Matrix(1, 1) << 2.0).finished();
    if (H3) *H3 = (Matrix(1, 1) << 3.0).finished();
    if (H4) *H4 = (Matrix(1, 1) << 4.0).finished();
    return (Vector(1) << x1 + 2.0 * x2 + 3.0 * x3 + 4.0 * x4).finished();
  }

  Key key1() const { return key<1>(); }  // Test that we can use key<> template
};

/* ************************************ */
TEST(NonlinearFactor, NoiseModelFactorN) {
  TestFactorN tf;
  Values tv;
  tv.insert(X(1), double((1.0)));
  tv.insert(X(2), double((2.0)));
  tv.insert(X(3), double((3.0)));
  tv.insert(X(4), double((4.0)));
  EXPECT(assert_equal((Vector(1) << 30.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(0.5 * 30.0 * 30.0 / 4.0, tf.error(tv), 1e-9);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv)));
  LONGS_EQUAL((long)X(1), (long)jf.keys()[0]);
  LONGS_EQUAL((long)X(2), (long)jf.keys()[1]);
  LONGS_EQUAL((long)X(3), (long)jf.keys()[2]);
  LONGS_EQUAL((long)X(4), (long)jf.keys()[3]);
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 0.5).finished(), jf.getA(jf.begin())));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.0).finished(), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.5).finished(), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.0).finished(), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal((Vector)(Vector(1) << -0.5 * 30.).finished(), jf.getb()));

  // Test all evaluateError argument overloads to ensure backward compatibility
  Matrix H1_expected, H2_expected, H3_expected, H4_expected;
  Vector e_expected = tf.evaluateError(9, 8, 7, 6, H1_expected, H2_expected,
                                       H3_expected, H4_expected);

  std::unique_ptr<NoiseModelFactorN<double, double, double, double>> base_ptr(
      new TestFactorN(tf));
  Matrix H1, H2, H3, H4;
  EXPECT(assert_equal(e_expected, base_ptr->evaluateError(9, 8, 7, 6)));
  EXPECT(assert_equal(e_expected, base_ptr->evaluateError(9, 8, 7, 6, H1)));
  EXPECT(assert_equal(H1_expected, H1));
  EXPECT(assert_equal(e_expected,  //
                      base_ptr->evaluateError(9, 8, 7, 6, H1, H2)));
  EXPECT(assert_equal(H1_expected, H1));
  EXPECT(assert_equal(H2_expected, H2));
  EXPECT(assert_equal(e_expected,
                      base_ptr->evaluateError(9, 8, 7, 6, H1, H2, H3)));
  EXPECT(assert_equal(H1_expected, H1));
  EXPECT(assert_equal(H2_expected, H2));
  EXPECT(assert_equal(H3_expected, H3));
  EXPECT(assert_equal(e_expected,
                      base_ptr->evaluateError(9, 8, 7, 6, H1, H2, H3, H4)));
  EXPECT(assert_equal(H1_expected, H1));
  EXPECT(assert_equal(H2_expected, H2));
  EXPECT(assert_equal(H3_expected, H3));
  EXPECT(assert_equal(H4_expected, H4));

  // Test all functions/types for backwards compatibility

  static_assert(std::is_same<TestFactor4::X1, double>::value,
                "X1 type incorrect");
  EXPECT(assert_equal(tf.key3(), X(3)));


  // Test using `key<N>` and `ValueType<N>`
  static_assert(std::is_same<TestFactorN::ValueType<1>, double>::value,
                "ValueType<1> type incorrect");
  static_assert(std::is_same<TestFactorN::ValueType<2>, double>::value,
                "ValueType<2> type incorrect");
  static_assert(std::is_same<TestFactorN::ValueType<3>, double>::value,
                "ValueType<3> type incorrect");
  static_assert(std::is_same<TestFactorN::ValueType<4>, double>::value,
                "ValueType<4> type incorrect");
  static_assert(std::is_same<TestFactorN::Type1, double>::value,
                "TestFactorN::Type1 type incorrect");
  EXPECT(assert_equal(tf.key<1>(), X(1)));
  EXPECT(assert_equal(tf.key<2>(), X(2)));
  EXPECT(assert_equal(tf.key<3>(), X(3)));
  EXPECT(assert_equal(tf.key<4>(), X(4)));
  EXPECT(assert_equal(tf.key1(), X(1)));
}

/* ************************************************************************* */
TEST( NonlinearFactor, clone_rekey )
{
  shared_nlf init(new TestFactor4());
  EXPECT_LONGS_EQUAL((long)X(1), (long)init->keys()[0]);
  EXPECT_LONGS_EQUAL((long)X(2), (long)init->keys()[1]);
  EXPECT_LONGS_EQUAL((long)X(3), (long)init->keys()[2]);
  EXPECT_LONGS_EQUAL((long)X(4), (long)init->keys()[3]);

  // Standard clone
  shared_nlf actClone = init->clone();
  EXPECT(actClone.get() != init.get()); // Ensure different pointers
  EXPECT(assert_equal(*init, *actClone));

  // Re-key factor - clones with different keys
  KeyVector new_keys {X(5),X(6),X(7),X(8)};
  shared_nlf actRekey = init->rekey(new_keys);
  EXPECT(actRekey.get() != init.get()); // Ensure different pointers

  // Ensure init is unchanged
  EXPECT_LONGS_EQUAL((long)X(1), (long)init->keys()[0]);
  EXPECT_LONGS_EQUAL((long)X(2), (long)init->keys()[1]);
  EXPECT_LONGS_EQUAL((long)X(3), (long)init->keys()[2]);
  EXPECT_LONGS_EQUAL((long)X(4), (long)init->keys()[3]);

  // Check new keys
  EXPECT_LONGS_EQUAL((long)X(5), (long)actRekey->keys()[0]);
  EXPECT_LONGS_EQUAL((long)X(6), (long)actRekey->keys()[1]);
  EXPECT_LONGS_EQUAL((long)X(7), (long)actRekey->keys()[2]);
  EXPECT_LONGS_EQUAL((long)X(8), (long)actRekey->keys()[3]);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
