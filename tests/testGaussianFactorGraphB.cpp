/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactorGraphB.cpp
 *  @brief  Unit tests for Linear Factor Graph
 *  @author Christian Potthast
 **/

#include <tests/smallExample.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

#include <string.h>
#include <iostream>

using namespace std;
using namespace gtsam;
using namespace example;

double tol=1e-5;

using symbol_shorthand::X;
using symbol_shorthand::L;

static auto kUnit2 = noiseModel::Unit::Create(2);

/* ************************************************************************* */
TEST( GaussianFactorGraph, equals ) {

  GaussianFactorGraph fg = createGaussianFactorGraph();
  GaussianFactorGraph fg2 = createGaussianFactorGraph();
  EXPECT(fg.equals(fg2));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, error ) {
  GaussianFactorGraph fg = createGaussianFactorGraph();
  VectorValues cfg = createZeroDelta();

  // note the error is the same as in testNonlinearFactorGraph as a
  // zero delta config in the linear graph is equivalent to noisy in
  // non-linear, which is really linear under the hood
  double actual = fg.error(cfg);
  DOUBLES_EQUAL( 5.625, actual, 1e-9 );
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminateOne_x1) {
  GaussianFactorGraph fg = createGaussianFactorGraph();

  GaussianConditional::shared_ptr conditional;
  auto result = fg.eliminatePartialSequential(Ordering{X(1)});
  conditional = result.first->front();

  // create expected Conditional Gaussian
  Matrix I = 15 * I_2x2, R11 = I, S12 = -0.111111 * I, S13 = -0.444444 * I;
  Vector d = Vector2(-0.133333, -0.0222222);
  GaussianConditional expected(X(1), 15 * d, R11, L(1), S12, X(2), S13);

  EXPECT(assert_equal(expected, *conditional, tol));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminateOne_x2) {
  Ordering ordering;
  ordering += X(2), L(1), X(1);
  GaussianFactorGraph fg = createGaussianFactorGraph();
  auto actual = EliminateQR(fg, Ordering{X(2)}).first;

  // create expected Conditional Gaussian
  double sigma = 0.0894427;
  Matrix I = I_2x2 / sigma, R11 = I, S12 = -0.2 * I, S13 = -0.8 * I;
  Vector d = Vector2(0.2, -0.14) / sigma;
  GaussianConditional expected(X(2), d, R11, L(1), S12, X(1), S13, kUnit2);

  EXPECT(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminateOne_l1) {
  Ordering ordering;
  ordering += L(1), X(1), X(2);
  GaussianFactorGraph fg = createGaussianFactorGraph();
  auto actual = EliminateQR(fg, Ordering{L(1)}).first;

  // create expected Conditional Gaussian
  double sigma = sqrt(2.0) / 10.;
  Matrix I = I_2x2 / sigma, R11 = I, S12 = -0.5 * I, S13 = -0.5 * I;
  Vector d = Vector2(-0.1, 0.25) / sigma;
  GaussianConditional expected(L(1), d, R11, X(1), S12, X(2), S13, kUnit2);

  EXPECT(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminateOne_x1_fast) {
  GaussianFactorGraph fg = createGaussianFactorGraph();
  const auto [conditional, remaining] = EliminateQR(fg, Ordering{X(1)});

  // create expected Conditional Gaussian
  Matrix I = 15 * I_2x2, R11 = I, S12 = -0.111111 * I, S13 = -0.444444 * I;
  Vector d = Vector2(-0.133333, -0.0222222);
  GaussianConditional expected(X(1), 15 * d, R11, L(1), S12, X(2), S13, kUnit2);

  // Create expected remaining new factor
  JacobianFactor expectedFactor(
      L(1), (Matrix(4, 2) << 6.87184, 0, 0, 6.87184, 0, 0, 0, 0).finished(),
      X(2),
      (Matrix(4, 2) << -5.25494, 0, 0, -5.25494, -7.27607, 0, 0, -7.27607)
          .finished(),
      (Vector(4) << -1.21268, 1.73817, -0.727607, 1.45521).finished(),
      noiseModel::Unit::Create(4));

  EXPECT(assert_equal(expected, *conditional, tol));
  EXPECT(assert_equal(expectedFactor, *remaining, tol));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminateOne_x2_fast) {
  GaussianFactorGraph fg = createGaussianFactorGraph();
  auto actual = EliminateQR(fg, Ordering{X(2)}).first;

  // create expected Conditional Gaussian
  double sigma = 0.0894427;
  Matrix I = I_2x2 / sigma, R11 = -I, S12 = 0.2 * I, S13 = 0.8 * I;
  Vector d = Vector2(-0.2, 0.14) / sigma;
  GaussianConditional expected(X(2), d, R11, L(1), S12, X(1), S13, kUnit2);

  EXPECT(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminateOne_l1_fast) {
  GaussianFactorGraph fg = createGaussianFactorGraph();
  auto actual = EliminateQR(fg, Ordering{L(1)}).first;

  // create expected Conditional Gaussian
  double sigma = sqrt(2.0) / 10.;
  Matrix I = I_2x2 / sigma, R11 = -I, S12 = 0.5 * I, S13 = 0.5 * I;
  Vector d = Vector2(0.1, -0.25) / sigma;
  GaussianConditional expected(L(1), d, R11, X(1), S12, X(2), S13, kUnit2);

  EXPECT(assert_equal(expected, *actual, tol));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, copying) {
  // Create a graph
  GaussianFactorGraph actual = createGaussianFactorGraph();

  // Copy the graph !
  GaussianFactorGraph copy = actual;

  // now eliminate the copy
  GaussianBayesNet actual1 = *copy.eliminateSequential();

  // Create the same graph, but not by copying
  GaussianFactorGraph expected = createGaussianFactorGraph();

  // and check that original is still the same graph
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, CONSTRUCTOR_GaussianBayesNet) {
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // render with a given ordering
  GaussianBayesNet CBN = *fg.eliminateSequential();

  // True GaussianFactorGraph
  GaussianFactorGraph fg2(CBN);
  GaussianBayesNet CBN2 = *fg2.eliminateSequential();
  EXPECT(assert_equal(CBN, CBN2));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, optimize_Cholesky) {
  // create a graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // optimize the graph
  VectorValues actual = fg.optimize(EliminateCholesky);

  // verify
  VectorValues expected = createCorrectDelta();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, optimize_QR )
{
  // create a graph
  GaussianFactorGraph fg = createGaussianFactorGraph();

  // optimize the graph
  VectorValues actual = fg.optimize(EliminateQR);

  // verify
  VectorValues expected = createCorrectDelta();
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, combine) {
  // create a test graph
  GaussianFactorGraph fg1 = createGaussianFactorGraph();

  // create another factor graph
  GaussianFactorGraph fg2 = createGaussianFactorGraph();

  // get sizes
  size_t size1 = fg1.size();
  size_t size2 = fg2.size();

  // combine them
  fg1.push_back(fg2);

  EXPECT(size1 + size2 == fg1.size());
}

/* ************************************************************************* */
// print a vector of ints if needed for debugging
void print(vector<int> v) {
  for (size_t k = 0; k < v.size(); k++) cout << v[k] << " ";
  cout << endl;
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, createSmoother) {
  GaussianFactorGraph fg1 = createSmoother(2);
  LONGS_EQUAL(3, fg1.size());
  GaussianFactorGraph fg2 = createSmoother(3);
  LONGS_EQUAL(5, fg2.size());
}

/* ************************************************************************* */
double error(const VectorValues& x) {
  GaussianFactorGraph fg = createGaussianFactorGraph();
  return fg.error(x);
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, multiplication) {
  GaussianFactorGraph A = createGaussianFactorGraph();
  VectorValues x = createCorrectDelta();
  Errors actual = A * x;
  Errors expected;
  expected.push_back(Vector2(-1.0, -1.0));
  expected.push_back(Vector2(2.0, -1.0));
  expected.push_back(Vector2(0.0, 1.0));
  expected.push_back(Vector2(-1.0, 1.5));
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
// Extra test on elimination prompted by Michael's email to Frank 1/4/2010
TEST(GaussianFactorGraph, elimination) {
  // Create Gaussian Factor Graph
  GaussianFactorGraph fg;
  Matrix Ap = I_1x1, An = I_1x1 * -1;
  Vector b = (Vector(1) << 0.0).finished();
  SharedDiagonal sigma = noiseModel::Isotropic::Sigma(1, 2.0);
  fg.emplace_shared<JacobianFactor>(X(1), An, X(2), Ap, b, sigma);
  fg.emplace_shared<JacobianFactor>(X(1), Ap, b, sigma);
  fg.emplace_shared<JacobianFactor>(X(2), Ap, b, sigma);

  // Eliminate
  Ordering ordering;
  ordering += X(1), X(2);
  GaussianBayesNet bayesNet = *fg.eliminateSequential();

  // Check matrix
  const auto [R, d] = bayesNet.matrix();
  Matrix expected =
      (Matrix(2, 2) << 0.707107, -0.353553, 0.0, 0.612372).finished();
  Matrix expected2 =
      (Matrix(2, 2) << 0.707107, -0.353553, 0.0, -0.612372).finished();
  EXPECT(assert_equal(expected, R, 1e-6));
  EXPECT(equal_with_abs_tol(expected, R, 1e-6) ||
         equal_with_abs_tol(expected2, R, 1e-6));
}

/* ************************************************************************* */
// Tests ported from ConstrainedGaussianFactorGraph
/* ************************************************************************* */
TEST(GaussianFactorGraph, constrained_simple) {
  // get a graph with a constraint in it
  GaussianFactorGraph fg = createSimpleConstraintGraph();
  EXPECT(hasConstraints(fg));

  // eliminate and solve
  VectorValues actual = fg.eliminateSequential()->optimize();

  // verify
  VectorValues expected = createSimpleConstraintValues();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, constrained_single) {
  // get a graph with a constraint in it
  GaussianFactorGraph fg = createSingleConstraintGraph();
  EXPECT(hasConstraints(fg));

  // eliminate and solve
  VectorValues actual = fg.eliminateSequential()->optimize();

  // verify
  VectorValues expected = createSingleConstraintValues();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, constrained_multi1) {
  // get a graph with a constraint in it
  GaussianFactorGraph fg = createMultiConstraintGraph();
  EXPECT(hasConstraints(fg));

  // eliminate and solve
  VectorValues actual = fg.eliminateSequential()->optimize();

  // verify
  VectorValues expected = createMultiConstraintValues();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */

static SharedDiagonal model = noiseModel::Isotropic::Sigma(2,1);

/* ************************************************************************* */
TEST(GaussianFactorGraph, replace)
{
  Ordering ord; ord += X(1),X(2),X(3),X(4),X(5),X(6);
  SharedDiagonal noise(noiseModel::Isotropic::Sigma(3, 1.0));

  GaussianFactorGraph::sharedFactor f1(new JacobianFactor(
      X(1), I_3x3, X(2), I_3x3, Z_3x1, noise));
  GaussianFactorGraph::sharedFactor f2(new JacobianFactor(
      X(2), I_3x3, X(3), I_3x3, Z_3x1, noise));
  GaussianFactorGraph::sharedFactor f3(new JacobianFactor(
      X(3), I_3x3, X(4), I_3x3, Z_3x1, noise));
  GaussianFactorGraph::sharedFactor f4(new JacobianFactor(
      X(5), I_3x3, X(6), I_3x3, Z_3x1, noise));

  GaussianFactorGraph actual;
  actual.push_back(f1);
  actual.push_back(f2);
  actual.push_back(f3);
  actual.replace(0, f4);

  GaussianFactorGraph expected;
  expected.push_back(f4);
  expected.push_back(f2);
  expected.push_back(f3);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, hasConstraints)
{
  FactorGraph<GaussianFactor> fgc1 = createMultiConstraintGraph();
  EXPECT(hasConstraints(fgc1));

  FactorGraph<GaussianFactor> fgc2 = createSimpleConstraintGraph() ;
  EXPECT(hasConstraints(fgc2));

  GaussianFactorGraph fg = createGaussianFactorGraph();
  EXPECT(!hasConstraints(fg));
}

#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/sam/RangeFactor.h>

/* ************************************************************************* */
TEST( GaussianFactorGraph, conditional_sigma_failure) {
  // This system derives from a failure case in DDF in which a Bayes Tree
  // has non-unit sigmas for conditionals in the Bayes Tree, which
  // should never happen by construction

  // Reason for the failure: using Vector_() is dangerous as having a non-float gets set to zero, resulting in constraints
  gtsam::Key xC1 = 0, l32 = 1, l41 = 2;

  // noisemodels at nonlinear level
  gtsam::SharedNoiseModel priorModel = noiseModel::Diagonal::Sigmas((Vector(6) << 0.05, 0.05, 3.0, 0.2, 0.2, 0.2).finished());
  gtsam::SharedNoiseModel measModel = kUnit2;
  gtsam::SharedNoiseModel elevationModel = noiseModel::Isotropic::Sigma(1, 3.0);

  double fov = 60; // degrees
  int imgW = 640; // pixels
  int imgH = 480; // pixels
  gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(fov, imgW, imgH));

  typedef GenericProjectionFactor<Pose3, Point3> ProjectionFactor;

  double relElevation = 6;

  Values initValues;
  initValues.insert(xC1,
      Pose3(Rot3(
          -1.,           0.0,  1.2246468e-16,
          0.0,             1.,           0.0,
          -1.2246468e-16,           0.0,            -1.),
          Point3(0.511832102, 8.42819594, 5.76841725)));
  initValues.insert(l32,  Point3(0.364081507, 6.89766221, -0.231582751) );
  initValues.insert(l41,  Point3(1.61051523, 6.7373052, -0.231582751)   );

  NonlinearFactorGraph factors;
  factors.addPrior(xC1,
      Pose3(Rot3(
          -1.,           0.0,  1.2246468e-16,
          0.0,             1.,           0.0,
          -1.2246468e-16,           0.0,            -1),
          Point3(0.511832102, 8.42819594, 5.76841725)), priorModel);
  factors += ProjectionFactor(Point2(333.648615, 98.61535), measModel, xC1, l32, K);
  factors += ProjectionFactor(Point2(218.508, 83.8022039), measModel, xC1, l41, K);
  factors += RangeFactor<Pose3,Point3>(xC1, l32, relElevation, elevationModel);
  factors += RangeFactor<Pose3,Point3>(xC1, l41, relElevation, elevationModel);

  // Check that sigmas are correct (i.e., unit)
  GaussianFactorGraph lfg = *factors.linearize(initValues);

  GaussianBayesTree actBT = *lfg.eliminateMultifrontal();

  // Check that all sigmas in an unconstrained bayes tree are set to one
  for (const auto& [key, clique]: actBT.nodes()) {
    GaussianConditional::shared_ptr conditional = clique->conditional();
    //size_t dim = conditional->rows();
    //EXPECT(assert_equal(gtsam::Vector::Ones(dim), conditional->get_model()->sigmas(), tol));
    EXPECT(!conditional->get_model());
  }
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
