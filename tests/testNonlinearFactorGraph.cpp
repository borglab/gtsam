/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testNonlinearFactorGraph.cpp
 * @brief   Unit tests for Non-Linear Factor NonlinearFactorGraph
 * @brief   testNonlinearFactorGraph
 * @author  Carlos Nieto
 * @author  Christian Potthast
 * @author  Frank Dellaert
 */

#include <gtsam/base/Testable.h>
#include <gtsam/base/Matrix.h>
#include <tests/smallExample.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
using namespace boost::assign;

/*STL/C++*/
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace example;

using symbol_shorthand::X;
using symbol_shorthand::L;

/* ************************************************************************* */
TEST( NonlinearFactorGraph, equals )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  NonlinearFactorGraph fg2 = createNonlinearFactorGraph();
  CHECK( fg.equals(fg2) );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, error )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  Values c1 = createValues();
  double actual1 = fg.error(c1);
  DOUBLES_EQUAL( 0.0, actual1, 1e-9 );

  Values c2 = createNoisyValues();
  double actual2 = fg.error(c2);
  DOUBLES_EQUAL( 5.625, actual2, 1e-9 );
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, keys )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  KeySet actual = fg.keys();
  LONGS_EQUAL(3, (long)actual.size());
  KeySet::const_iterator it = actual.begin();
  LONGS_EQUAL((long)L(1), (long)*(it++));
  LONGS_EQUAL((long)X(1), (long)*(it++));
  LONGS_EQUAL((long)X(2), (long)*(it++));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, GET_ORDERING)
{
  Ordering expected; expected += L(1), X(2), X(1); // For starting with l1,x1,x2
  NonlinearFactorGraph nlfg = createNonlinearFactorGraph();
  Ordering actual = Ordering::Colamd(nlfg);
  EXPECT(assert_equal(expected,actual));

  // Constrained ordering - put x2 at the end
  Ordering expectedConstrained; expectedConstrained += L(1), X(1), X(2);
  FastMap<Key, int> constraints;
  constraints[X(2)] = 1;
  Ordering actualConstrained = Ordering::ColamdConstrained(nlfg, constraints);
  EXPECT(assert_equal(expectedConstrained, actualConstrained));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, probPrime )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  Values cfg = createValues();

  // evaluate the probability of the factor graph
  double actual = fg.probPrime(cfg);
  double expected = 1.0;
  DOUBLES_EQUAL(expected,actual,0);
}

/* ************************************************************************* */
TEST(NonlinearFactorGraph, ProbPrime2) {
  NonlinearFactorGraph fg;
  fg.emplace_shared<PriorFactor<double>>(1, 0.0,
                                         noiseModel::Isotropic::Sigma(1, 1.0));

  Values values;
  values.insert(1, 1.0);

  // The prior factor squared error is: 0.5.
  EXPECT_DOUBLES_EQUAL(0.5, fg.error(values), 1e-12);

  // The probability value is: exp^(-factor_error) / sqrt(2 * PI)
  // Ignore the denominator and we get: exp^(-factor_error) = exp^(-0.5)
  double expected = exp(-0.5);
  EXPECT_DOUBLES_EQUAL(expected, fg.probPrime(values), 1e-12);
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, linearize )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  Values initial = createNoisyValues();
  GaussianFactorGraph linearFG = *fg.linearize(initial);
  GaussianFactorGraph expected = createGaussianFactorGraph();
  CHECK(assert_equal(expected,linearFG)); // Needs correct linearizations
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, clone )
{
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  NonlinearFactorGraph actClone = fg.clone();
  EXPECT(assert_equal(fg, actClone));
  for (size_t i=0; i<fg.size(); ++i)
    EXPECT(fg[i] != actClone[i]);
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, rekey )
{
  NonlinearFactorGraph init = createNonlinearFactorGraph();
  map<Key,Key> rekey_mapping;
  rekey_mapping.insert(make_pair(L(1), L(4)));
  NonlinearFactorGraph actRekey = init.rekey(rekey_mapping);

  // ensure deep clone
  LONGS_EQUAL((long)init.size(), (long)actRekey.size());
  for (size_t i=0; i<init.size(); ++i)
      EXPECT(init[i] != actRekey[i]);

  NonlinearFactorGraph expRekey;
  // original measurements
  expRekey.push_back(init[0]);
  expRekey.push_back(init[1]);

  // updated measurements
  Point2 z3(0, -1),  z4(-1.5, -1.);
  SharedDiagonal sigma0_2 = noiseModel::Isotropic::Sigma(2,0.2);
  expRekey += simulated2D::Measurement(z3, sigma0_2, X(1), L(4));
  expRekey += simulated2D::Measurement(z4, sigma0_2, X(2), L(4));

  EXPECT(assert_equal(expRekey, actRekey));
}

/* ************************************************************************* */
TEST( NonlinearFactorGraph, symbolic )
{
  NonlinearFactorGraph graph = createNonlinearFactorGraph();

  SymbolicFactorGraph expected;
  expected.push_factor(X(1));
  expected.push_factor(X(1), X(2));
  expected.push_factor(X(1), L(1));
  expected.push_factor(X(2), L(1));

  SymbolicFactorGraph actual = *graph.symbolic();

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(NonlinearFactorGraph, UpdateCholesky) {
  NonlinearFactorGraph fg = createNonlinearFactorGraph();
  Values initial = createNoisyValues();

  // solve conventionally
  GaussianFactorGraph linearFG = *fg.linearize(initial);
  auto delta = linearFG.optimizeDensely();
  auto expected = initial.retract(delta);

  // solve with new method
  EXPECT(assert_equal(expected, fg.updateCholesky(initial)));

  // solve with Ordering
  Ordering ordering;
  ordering += L(1), X(2), X(1);
  EXPECT(assert_equal(expected, fg.updateCholesky(initial, ordering)));

  // solve with new method, heavily damped
  auto dampen = [](const HessianFactor::shared_ptr& hessianFactor) {
    auto iterator = hessianFactor->begin();
    for (; iterator != hessianFactor->end(); iterator++) {
      const auto index = std::distance(hessianFactor->begin(), iterator);
      auto block = hessianFactor->info().diagonalBlock(index);
      for (int j = 0; j < block.rows(); j++) {
        block(j, j) += 1e9;
      }
    }
  };
  EXPECT(assert_equal(initial, fg.updateCholesky(initial, dampen), 1e-6));
}

/* ************************************************************************* */
// Example from issue #452 which threw an ILS error. The reason was a very 
// weak prior on heading, which was tightened, and the ILS disappeared.
TEST(testNonlinearFactorGraph, eliminate) {
  // Linearization point
  Pose2 T11(0, 0, 0);
  Pose2 T12(1, 0, 0);
  Pose2 T21(0, 1, 0);
  Pose2 T22(1, 1, 0);

  // Factor graph
  auto graph = NonlinearFactorGraph();

  // Priors
  auto prior = noiseModel::Isotropic::Sigma(3, 1);
  graph.addPrior(11, T11, prior);
  graph.addPrior(21, T21, prior);

  // Odometry
  auto model = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.3));
  graph.add(BetweenFactor<Pose2>(11, 12, T11.between(T12), model));
  graph.add(BetweenFactor<Pose2>(21, 22, T21.between(T22), model));

  // Range factor
  auto model_rho = noiseModel::Isotropic::Sigma(1, 0.01);
  graph.add(RangeFactor<Pose2>(12, 22, 1.0, model_rho));

  Values values;
  values.insert(11, T11.retract(Vector3(0.1,0.2,0.3)));
  values.insert(12, T12);
  values.insert(21, T21);
  values.insert(22, T22);
  auto linearized = graph.linearize(values);

  // Eliminate
  Ordering ordering;
  ordering += 11, 21, 12, 22;
  auto bn = linearized->eliminateSequential(ordering);
  EXPECT_LONGS_EQUAL(4, bn->size());
}

/* ************************************************************************* */
TEST(testNonlinearFactorGraph, addPrior) {
  Key k(0);

  // Factor graph.
  auto graph = NonlinearFactorGraph();

  // Add a prior factor for key k.
  auto model_double = noiseModel::Isotropic::Sigma(1, 1);
  graph.addPrior<double>(k, 10, model_double);

  // Assert the graph has 0 error with the correct values.
  Values values;
  values.insert(k, 10.0);
  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-16);

  // Assert the graph has some error with incorrect values.
  values.clear();
  values.insert(k, 11.0);
  EXPECT(0 != graph.error(values));

  // Clear the factor graph and values.
  values.clear();
  graph.erase(graph.begin(), graph.end());

  // Add a Pose3 prior to the factor graph. Use a gaussian noise model by
  // providing the covariance matrix.
  Eigen::DiagonalMatrix<double, 6, 6> covariance_pose3;
  covariance_pose3.setIdentity();
  Pose3 pose{Rot3(), Point3(0, 0, 0)};
  graph.addPrior(k, pose, covariance_pose3);

  // Assert the graph has 0 error with the correct values.
  values.insert(k, pose);
  EXPECT_DOUBLES_EQUAL(0, graph.error(values), 1e-16);

  // Assert the graph has some error with incorrect values.
  values.clear();
  Pose3 pose_incorrect{Rot3::RzRyRx(-M_PI, M_PI, -M_PI / 8), Point3(1, 2, 3)};
  values.insert(k, pose_incorrect);
  EXPECT(0 != graph.error(values));
}

/* ************************************************************************* */
TEST(NonlinearFactorGraph, printErrors)
{
  const NonlinearFactorGraph fg = createNonlinearFactorGraph();
  const Values c = createValues();

  // Test that it builds with default parameters.
  // We cannot check the output since (at present) output is fixed to std::cout.
  fg.printErrors(c);

  // Second round: using callback filter to check that we actually visit all factors:
  std::vector<bool> visited;
  visited.assign(fg.size(), false);
  const auto testFilter =
      [&](const gtsam::Factor *f, double error, size_t index) {
        EXPECT(f!=nullptr);
        EXPECT(error>=.0);
        visited.at(index)=true;
        return false; // do not print
      };
  fg.printErrors(c,"Test graph: ", gtsam::DefaultKeyFormatter,testFilter);

  for (bool visit : visited) EXPECT(visit==true);
}

/* ************************************************************************* */
TEST(NonlinearFactorGraph, dot) {
  string expected =
      "graph {\n"
      "  size=\"5,5\";\n"
      "\n"
      "  var7782220156096217089[label=\"l1\"];\n"
      "  var8646911284551352321[label=\"x1\"];\n"
      "  var8646911284551352322[label=\"x2\"];\n"
      "\n"
      "  factor0[label=\"\", shape=point];\n"
      "  var8646911284551352321--factor0;\n"
      "  factor1[label=\"\", shape=point];\n"
      "  var8646911284551352321--factor1;\n"
      "  var8646911284551352322--factor1;\n"
      "  factor2[label=\"\", shape=point];\n"
      "  var8646911284551352321--factor2;\n"
      "  var7782220156096217089--factor2;\n"
      "  factor3[label=\"\", shape=point];\n"
      "  var8646911284551352322--factor3;\n"
      "  var7782220156096217089--factor3;\n"
      "}\n";

  const NonlinearFactorGraph fg = createNonlinearFactorGraph();
  string actual = fg.dot();
  EXPECT(actual == expected);
}

/* ************************************************************************* */
TEST(NonlinearFactorGraph, dot_extra) {
  string expected =
      "graph {\n"
      "  size=\"5,5\";\n"
      "\n"
      "  var7782220156096217089[label=\"l1\", pos=\"0,0!\"];\n"
      "  var8646911284551352321[label=\"x1\", pos=\"1,0!\"];\n"
      "  var8646911284551352322[label=\"x2\", pos=\"1,1.5!\"];\n"
      "\n"
      "  factor0[label=\"\", shape=point];\n"
      "  var8646911284551352321--factor0;\n"
      "  factor1[label=\"\", shape=point];\n"
      "  var8646911284551352321--factor1;\n"
      "  var8646911284551352322--factor1;\n"
      "  factor2[label=\"\", shape=point];\n"
      "  var8646911284551352321--factor2;\n"
      "  var7782220156096217089--factor2;\n"
      "  factor3[label=\"\", shape=point];\n"
      "  var8646911284551352322--factor3;\n"
      "  var7782220156096217089--factor3;\n"
      "}\n";

  const NonlinearFactorGraph fg = createNonlinearFactorGraph();
  const Values c = createValues();

  stringstream ss;
  fg.dot(ss, c);
  EXPECT(ss.str() == expected);
}

/* ************************************************************************* */
template <class VALUE>
class MyPrior : public gtsam::NoiseModelFactorN<VALUE> {
 private:
  VALUE prior_;

 public:
  MyPrior(gtsam::Key key, const VALUE &prior,
          const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactorN<VALUE>(model, key), prior_(prior) {}

  gtsam::Vector evaluateError(
      const VALUE &val,
      boost::optional<gtsam::Matrix &> H = boost::none) const override {
    if (H)
      (*H) = gtsam::Matrix::Identity(gtsam::traits<VALUE>::GetDimension(val),
                                     gtsam::traits<VALUE>::GetDimension(val));
    // manifold equivalent of z-x -> Local(x,z)
    return -gtsam::traits<VALUE>::Local(val, prior_);
  }

  virtual gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new MyPrior<VALUE>(*this)));
  }
};

template <class VALUE>
class MyPriorPrint : public gtsam::NoiseModelFactorN<VALUE> {
 private:
  VALUE prior_;

 public:
  MyPriorPrint(gtsam::Key key, const VALUE &prior,
               const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactorN<VALUE>(model, key), prior_(prior) {}

  gtsam::Vector evaluateError(
      const VALUE &val,
      boost::optional<gtsam::Matrix &> H = boost::none) const override {
    if (H)
      (*H) = gtsam::Matrix::Identity(gtsam::traits<VALUE>::GetDimension(val),
                                     gtsam::traits<VALUE>::GetDimension(val));
    // manifold equivalent of z-x -> Local(x,z)
    auto error = -gtsam::traits<VALUE>::Local(val, prior_);
    val.print();
    prior_.print();
    return error;
  }

  virtual gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new MyPriorPrint<VALUE>(*this)));
  }
};

TEST(NonlinearFactorGraph, NoPrintSideEffects) {
  NonlinearFactorGraph fg;
  Values vals;
  const auto model = noiseModel::Unit::Create(3);
  fg.emplace_shared<MyPrior<Pose2>>(0, Pose2(0, 0, 0), model);
  vals.insert(0, Pose2(1, 1, 1));

  NonlinearFactorGraph fg_print;
  Values vals_print;
  fg_print.emplace_shared<MyPriorPrint<Pose2>>(0, Pose2(0, 0, 0), model);
  vals_print.insert(0, Pose2(1, 1, 1));

  std::cout << "Without Prints:" << std::endl;
  GaussNewtonOptimizer optimizer(fg, vals);
  optimizer.optimize().print();

  std::cout << "With Prints:" << std::endl;
  GaussNewtonOptimizer optimizer_print(fg_print, vals_print);
  optimizer_print.optimize().print();
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
