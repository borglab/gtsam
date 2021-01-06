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
class TestFactor4 : public NoiseModelFactor4<double, double, double, double> {
public:
  typedef NoiseModelFactor4<double, double, double, double> Base;
  TestFactor4() : Base(noiseModel::Diagonal::Sigmas((Vector(1) << 2.0).finished()), X(1), X(2), X(3), X(4)) {}

  Vector
    evaluateError(const double& x1, const double& x2, const double& x3, const double& x4,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none) const override {
    if(H1) {
      *H1 = (Matrix(1, 1) << 1.0).finished();
      *H2 = (Matrix(1, 1) << 2.0).finished();
      *H3 = (Matrix(1, 1) << 3.0).finished();
      *H4 = (Matrix(1, 1) << 4.0).finished();
    }
    return (Vector(1) << x1 + x2 + x3 + x4).finished();
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
  EXPECT(assert_equal((Vector(1) << 10.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(25.0/2.0, tf.error(tv), 1e-9);
  JacobianFactor jf(*boost::dynamic_pointer_cast<JacobianFactor>(tf.linearize(tv)));
  LONGS_EQUAL((long)X(1), (long)jf.keys()[0]);
  LONGS_EQUAL((long)X(2), (long)jf.keys()[1]);
  LONGS_EQUAL((long)X(3), (long)jf.keys()[2]);
  LONGS_EQUAL((long)X(4), (long)jf.keys()[3]);
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 0.5).finished(), jf.getA(jf.begin())));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.0).finished(), jf.getA(jf.begin()+1)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 1.5).finished(), jf.getA(jf.begin()+2)));
  EXPECT(assert_equal((Matrix)(Matrix(1, 1) << 2.0).finished(), jf.getA(jf.begin()+3)));
  EXPECT(assert_equal((Vector)(Vector(1) << -5.0).finished(), jf.getb()));
}

/* ************************************************************************* */
class TestFactor5 : public NoiseModelFactor5<double, double, double, double, double> {
public:
  typedef NoiseModelFactor5<double, double, double, double, double> Base;
  TestFactor5() : Base(noiseModel::Diagonal::Sigmas((Vector(1) << 2.0).finished()), X(1), X(2), X(3), X(4), X(5)) {}

  Vector
    evaluateError(const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none) const override {
    if(H1) {
      *H1 = (Matrix(1, 1) << 1.0).finished();
      *H2 = (Matrix(1, 1) << 2.0).finished();
      *H3 = (Matrix(1, 1) << 3.0).finished();
      *H4 = (Matrix(1, 1) << 4.0).finished();
      *H5 = (Matrix(1, 1) << 5.0).finished();
    }
    return (Vector(1) << x1 + x2 + x3 + x4 + x5).finished();
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
  EXPECT(assert_equal((Vector(1) << 15.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(56.25/2.0, tf.error(tv), 1e-9);
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
  EXPECT(assert_equal((Vector)(Vector(1) << -7.5).finished(), jf.getb()));
}

/* ************************************************************************* */
class TestFactor6 : public NoiseModelFactor6<double, double, double, double, double, double> {
public:
  typedef NoiseModelFactor6<double, double, double, double, double, double> Base;
  TestFactor6() : Base(noiseModel::Diagonal::Sigmas((Vector(1) << 2.0).finished()), X(1), X(2), X(3), X(4), X(5), X(6)) {}

  Vector
    evaluateError(const X1& x1, const X2& x2, const X3& x3, const X4& x4, const X5& x5, const X6& x6,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none,
        boost::optional<Matrix&> H3 = boost::none,
        boost::optional<Matrix&> H4 = boost::none,
        boost::optional<Matrix&> H5 = boost::none,
        boost::optional<Matrix&> H6 = boost::none) const override {
    if(H1) {
      *H1 = (Matrix(1, 1) << 1.0).finished();
      *H2 = (Matrix(1, 1) << 2.0).finished();
      *H3 = (Matrix(1, 1) << 3.0).finished();
      *H4 = (Matrix(1, 1) << 4.0).finished();
      *H5 = (Matrix(1, 1) << 5.0).finished();
      *H6 = (Matrix(1, 1) << 6.0).finished();
    }
    return (Vector(1) << x1 + x2 + x3 + x4 + x5 + x6).finished();
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
  EXPECT(assert_equal((Vector(1) << 21.0).finished(), tf.unwhitenedError(tv)));
  DOUBLES_EQUAL(110.25/2.0, tf.error(tv), 1e-9);
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
  EXPECT(assert_equal((Vector)(Vector(1) << -10.5).finished(), jf.getb()));

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
