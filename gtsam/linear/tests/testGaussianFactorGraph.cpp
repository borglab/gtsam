/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactorGraphUnordered.cpp
 *  @brief  Unit tests for Linear Factor Graph
 *  @author Christian Potthast
 *  @author Frank Dellaert
 *  @author Luca Carlone
 *  @author Richard Roberts
 **/

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/base/debug.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace std;
using namespace gtsam;

typedef std::tuple<size_t, size_t, double> SparseTriplet;
bool triplet_equal(SparseTriplet a, SparseTriplet b) {
  if (get<0>(a) == get<0>(b) && get<1>(a) == get<1>(b) &&
      get<2>(a) == get<2>(b)) return true;

  cout << "not equal:" << endl;
  cout << "\texpected: "
      "(" << get<0>(a) << ", " << get<1>(a) << ") = " << get<2>(a) << endl;
  cout << "\tactual:   "
      "(" << get<0>(b) << ", " << get<1>(b) << ") = " << get<2>(b) << endl;
  return false;
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, initialization) {
  // Create empty graph
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);

  fg.emplace_shared<JacobianFactor>(0, 10*I_2x2, -1.0*Vector::Ones(2), unit2);
  fg.emplace_shared<JacobianFactor>(0, -10*I_2x2,1, 10*I_2x2, Vector2(2.0, -1.0), unit2);
  fg.emplace_shared<JacobianFactor>(0, -5*I_2x2, 2, 5*I_2x2, Vector2(0.0, 1.0), unit2);
  fg.emplace_shared<JacobianFactor>(1, -5*I_2x2, 2, 5*I_2x2, Vector2(-1.0, 1.5), unit2);

  EXPECT_LONGS_EQUAL(4, (long)fg.size());

  // Test sparse, which takes a vector and returns a matrix, used in MATLAB
  // Note that this the augmented vector and the RHS is in column 7
  Matrix expectedIJS =
      (Matrix(3, 21) <<
      1., 2., 1., 2., 3., 4., 3., 4., 3., 4., 5., 6., 5., 6., 6., 7., 8., 7., 8., 7., 8.,
      1., 2., 7., 7., 1., 2., 3., 4., 7., 7., 1., 2., 5., 6., 7., 3., 4., 5., 6., 7., 7.,
      10., 10., -1., -1., -10., -10., 10., 10., 2., -1., -5., -5., 5., 5.,
        1., -5., -5., 5., 5., -1., 1.5).finished();
  Matrix actualIJS = fg.sparseJacobian_();
  EQUALITY(expectedIJS, actualIJS);
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, sparseJacobian) {
  // Create factor graph:
  // x1 x2 x3 x4 x5  b
  //  1  2  3  0  0  4
  //  5  6  7  0  0  8
  //  9 10  0 11 12 13
  //  0  0  0 14 15 16

  // Expected
  Matrix expected = (Matrix(16, 3) <<
      1., 1., 2.,
      1., 2., 4.,
      1., 3., 6.,
      2., 1.,10.,
      2., 2.,12.,
      2., 3.,14.,
      1., 6., 8.,
      2., 6.,16.,
      3., 1.,18.,
      3., 2.,20.,
      3., 4.,22.,
      3., 5.,24.,
      4., 4.,28.,
      4., 5.,30.,
      3., 6.,26.,
      4., 6.,32.).finished();

  // expected: in matlab format - NOTE the transpose!)
  Matrix expectedMatlab = expected.transpose();

  GaussianFactorGraph gfg;
  SharedDiagonal model = noiseModel::Isotropic::Sigma(2, 0.5);
  const Key x123 = 0, x45 = 1;
  gfg.add(x123, (Matrix(2, 3) << 1, 2, 3, 5, 6, 7).finished(),
          Vector2(4, 8), model);
  gfg.add(x123, (Matrix(2, 3) << 9, 10, 0, 0, 0, 0).finished(),
          x45,  (Matrix(2, 2) << 11, 12, 14, 15.).finished(),
          Vector2(13, 16), model);

  Matrix actual = gfg.sparseJacobian_();

  EXPECT(assert_equal(expectedMatlab, actual));

  // SparseTriplets
  auto boostActual = gfg.sparseJacobian();
  // check the triplets size...
  EXPECT_LONGS_EQUAL(16, boostActual.size());
  // check content
  for (int i = 0; i < 16; i++) {
    EXPECT(triplet_equal(
        SparseTriplet(expected(i, 0) - 1, expected(i, 1) - 1, expected(i, 2)),
        boostActual.at(i)));
  }
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, matrices) {
  // Create factor graph:
  // x1 x2 x3 x4 x5  b
  //  1  2  3  0  0  4
  //  5  6  7  0  0  8
  //  9 10  0 11 12 13
  //  0  0  0 14 15 16

  Matrix A00 = (Matrix(2, 3) << 1, 2, 3, 5, 6, 7).finished();
  Matrix A10 = (Matrix(2, 3) << 9, 10, 0, 0, 0, 0).finished();
  Matrix A11 = (Matrix(2, 2) << 11, 12, 14, 15).finished();

  GaussianFactorGraph gfg;
  SharedDiagonal model = noiseModel::Unit::Create(2);
  gfg.add(0, A00, Vector2(4., 8.), model);
  gfg.add(0, A10, 1, A11, Vector2(13., 16.), model);

  Matrix Ab(4, 6);
  Ab << 1, 2, 3, 0, 0, 4, 5, 6, 7, 0, 0, 8, 9, 10, 0, 11, 12, 13, 0, 0, 0, 14, 15, 16;

  // augmented versions
  EXPECT(assert_equal(Ab, gfg.augmentedJacobian()));
  EXPECT(assert_equal(Ab.transpose() * Ab, gfg.augmentedHessian()));

  // jacobian
  Matrix A = Ab.leftCols(Ab.cols() - 1);
  Vector b = Ab.col(Ab.cols() - 1);
  const auto [actualA, actualb] = gfg.jacobian();
  EXPECT(assert_equal(A, actualA));
  EXPECT(assert_equal(b, actualb));

  // hessian
  Matrix L = A.transpose() * A;
  Vector eta = A.transpose() * b;
  const auto [actualL, actualEta] = gfg.hessian();
  EXPECT(assert_equal(L, actualL));
  EXPECT(assert_equal(eta, actualEta));

  // hessianBlockDiagonal
  VectorValues expectLdiagonal;  // Make explicit that diagonal is sum-squares of columns
  expectLdiagonal.insert(0, Vector3(1 + 25 + 81, 4 + 36 + 100, 9 + 49));
  expectLdiagonal.insert(1, Vector2(121 + 196, 144 + 225));
  EXPECT(assert_equal(expectLdiagonal, gfg.hessianDiagonal()));

  // hessianBlockDiagonal
  map<Key, Matrix> actualBD = gfg.hessianBlockDiagonal();
  LONGS_EQUAL(2, actualBD.size());
  EXPECT(assert_equal(A00.transpose() * A00 + A10.transpose() * A10, actualBD[0]));
  EXPECT(assert_equal(A11.transpose() * A11, actualBD[1]));
}

/* ************************************************************************* */
/// Factor graph with 2 2D factors on 3 2D variables
static GaussianFactorGraph createSimpleGaussianFactorGraph() {
  GaussianFactorGraph fg;
  Key x1 = 2, x2 = 0, l1 = 1;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
  fg.emplace_shared<JacobianFactor>(x1, 10 * I_2x2, -1.0 * Vector::Ones(2), unit2);
  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg.emplace_shared<JacobianFactor>(x2, 10 * I_2x2, x1, -10 * I_2x2, Vector2(2.0, -1.0), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg.emplace_shared<JacobianFactor>(l1, 5 * I_2x2, x1, -5 * I_2x2, Vector2(0.0, 1.0), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg.emplace_shared<JacobianFactor>(x2, -5 * I_2x2, l1, 5 * I_2x2, Vector2(-1.0, 1.5), unit2);
  return fg;
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, gradient) {
  GaussianFactorGraph fg = createSimpleGaussianFactorGraph();

  // Construct expected gradient
  // 2*f(x) = 100*(x1+c[X(1)])^2 + 100*(x2-x1-[0.2;-0.1])^2 + 25*(l1-x1-[0.0;0.2])^2 +
  // 25*(l1-x2-[-0.2;0.3])^2
  // worked out: df/dx1 = 100*[0.1;0.1] + 100*[0.2;-0.1]) + 25*[0.0;0.2] = [10+20;10-10+5] = [30;5]
  VectorValues expected{{1, Vector2(5.0, -12.5)},
                        {2, Vector2(30.0, 5.0)},
                        {0, Vector2(-25.0, 17.5)}};

  // Check the gradient at delta=0
  VectorValues zero = VectorValues::Zero(expected);
  VectorValues actual = fg.gradient(zero);
  EXPECT(assert_equal(expected, actual));
  EXPECT(assert_equal(expected, fg.gradientAtZero()));

  // Check the gradient at the solution (should be zero)
  VectorValues solution = fg.optimize();
  VectorValues actual2 = fg.gradient(solution);
  EXPECT(assert_equal(VectorValues::Zero(solution), actual2));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, transposeMultiplication) {
  GaussianFactorGraph A = createSimpleGaussianFactorGraph();

  Errors e = {Vector2(0.0, 0.0), Vector2(15.0, 0.0), Vector2(0.0, -5.0),
              Vector2(-7.5, -5.0)};

  VectorValues expected;
  expected.insert(1, Vector2(-37.5, -50.0));
  expected.insert(2, Vector2(-150.0, 25.0));
  expected.insert(0, Vector2(187.5, 25.0));

  VectorValues actual = A.transposeMultiply(e);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminate_empty) {
  // eliminate an empty factor
  GaussianFactorGraph gfg;
  gfg.add(JacobianFactor());
  const auto [actualBN, remainingGFG] = gfg.eliminatePartialSequential(Ordering());

  // expected Bayes net is empty
  GaussianBayesNet expectedBN;

  // expected remaining graph should be the same as the original, still containing the empty factor
  GaussianFactorGraph expectedLF = gfg;

  // check if the result matches
  EXPECT(assert_equal(*actualBN, expectedBN));
  EXPECT(assert_equal(*remainingGFG, expectedLF));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, matrices2) {
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();
  const auto [A, b] = gfg.jacobian();
  const auto [AtA, eta] = gfg.hessian();
  EXPECT(assert_equal(A.transpose() * A, AtA));
  EXPECT(assert_equal(A.transpose() * b, eta));
  Matrix expectedAtA(6, 6);
  expectedAtA << 125, 0, -25, 0, -100, 0,  //
      0, 125, 0, -25, 0, -100,             //
      -25, 0, 50, 0, -25, 0,               //
      0, -25, 0, 50, 0, -25,               //
      -100, 0, -25, 0, 225, 0,             //
      0, -100, 0, -25, 0, 225;
  EXPECT(assert_equal(expectedAtA, AtA));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, multiplyHessianAdd) {
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();

  const VectorValues x{{0, Vector2(1, 2)}, {1, Vector2(3, 4)}, {2, Vector2(5, 6)}};

  VectorValues expected;
  expected.insert(0, Vector2(-450, -450));
  expected.insert(1, Vector2(0, 0));
  expected.insert(2, Vector2(950, 1050));

  VectorValues actual;
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(expected, actual));

  // now, do it with non-zero y
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(2 * expected, actual));
}

/* ************************************************************************* */
static GaussianFactorGraph createGaussianFactorGraphWithHessianFactor() {
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();
  gfg.emplace_shared<HessianFactor>(1, 2, 100 * I_2x2, Z_2x2, Vector2(0.0, 1.0),
                                    400 * I_2x2, Vector2(1.0, 1.0), 3.0);
  return gfg;
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, multiplyHessianAdd2) {
  GaussianFactorGraph gfg = createGaussianFactorGraphWithHessianFactor();

  // brute force
  const auto [AtA, eta] = gfg.hessian();
  Vector X(6);
  X << 1, 2, 3, 4, 5, 6;
  Vector Y(6);
  Y << -450, -450, 300, 400, 2950, 3450;
  EXPECT(assert_equal(Y, AtA * X));

  const VectorValues x {{0, Vector2(1, 2)}, {1, Vector2(3, 4)}, {2, Vector2(5, 6)}};
  VectorValues expected;
  expected.insert(0, Vector2(-450, -450));
  expected.insert(1, Vector2(300, 400));
  expected.insert(2, Vector2(2950, 3450));

  VectorValues actual;
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(expected, actual));

  // now, do it with non-zero y
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(2 * expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, matricesMixed) {
  GaussianFactorGraph gfg = createGaussianFactorGraphWithHessianFactor();
  const auto [A, b] = gfg.jacobian();  // incorrect !
  const auto [AtA, eta] = gfg.hessian();  // correct
  EXPECT(assert_equal(A.transpose() * A, AtA));
  Vector expected = -(Vector(6) << -25, 17.5, 5, -13.5, 29, 4).finished();
  EXPECT(assert_equal(expected, eta));
  EXPECT(assert_equal(A.transpose() * b, eta));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, gradientAtZero) {
  GaussianFactorGraph gfg = createGaussianFactorGraphWithHessianFactor();
  VectorValues expected;
  VectorValues actual = gfg.gradientAtZero();
  expected.insert(0, Vector2(-25, 17.5));
  expected.insert(1, Vector2(5, -13.5));
  expected.insert(2, Vector2(29, 4));
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, clone) {
  // 2 variables, frontal has dim=4
  VerticalBlockMatrix blockMatrix(KeyVector{4, 2, 1}, 4);
  blockMatrix.matrix() << 1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 0.1, 0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.2, 0.0,
      0.0, 3.0, 0.0, 4.0, 0.0, 0.3, 0.0, 0.0, 0.0, 3.0, 0.0, 4.0, 0.4;
  GaussianConditional cg(KeyVector{1, 2}, 1, blockMatrix);

  GaussianFactorGraph init_graph = createGaussianFactorGraphWithHessianFactor();
  init_graph.push_back(GaussianFactor::shared_ptr());  /// Add null factor
  init_graph.push_back(GaussianConditional(cg));

  GaussianFactorGraph exp_graph =
      createGaussianFactorGraphWithHessianFactor();   // Created separately
  exp_graph.push_back(GaussianFactor::shared_ptr());  /// Add null factor
  exp_graph.push_back(GaussianConditional(cg));

  GaussianFactorGraph actCloned = init_graph.clone();
  EXPECT(assert_equal(init_graph, actCloned));  // Same as the original version

  // Apply an in-place change to init_graph and compare
  JacobianFactor::shared_ptr jacFactor0 =
      std::dynamic_pointer_cast<JacobianFactor>(init_graph.at(0));
  CHECK(jacFactor0);
  jacFactor0->getA(jacFactor0->begin()) *= 7.;
  EXPECT(assert_inequal(init_graph, exp_graph));
  EXPECT(assert_equal(exp_graph, actCloned));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, negate) {
  GaussianFactorGraph init_graph = createGaussianFactorGraphWithHessianFactor();
  init_graph.push_back(GaussianFactor::shared_ptr());  /// Add null factor
  GaussianFactorGraph actNegation = init_graph.negate();
  GaussianFactorGraph expNegation;
  expNegation.push_back(init_graph.at(0)->negate());
  expNegation.push_back(init_graph.at(1)->negate());
  expNegation.push_back(init_graph.at(2)->negate());
  expNegation.push_back(init_graph.at(3)->negate());
  expNegation.push_back(init_graph.at(4)->negate());
  expNegation.push_back(GaussianFactor::shared_ptr());
  EXPECT(assert_equal(expNegation, actNegation));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, hessianDiagonal) {
  GaussianFactorGraph gfg = createGaussianFactorGraphWithHessianFactor();
  VectorValues expected;
  Matrix infoMatrix = gfg.hessian().first;
  Vector d = infoMatrix.diagonal();

  VectorValues actual = gfg.hessianDiagonal();
  expected.insert(0, d.segment<2>(0));
  expected.insert(1, d.segment<2>(2));
  expected.insert(2, d.segment<2>(4));
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, DenseSolve) {
  GaussianFactorGraph fg = createSimpleGaussianFactorGraph();
  VectorValues expected = fg.optimize();
  VectorValues actual = fg.optimizeDensely();
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, ProbPrime) {
  GaussianFactorGraph gfg;
  gfg.emplace_shared<JacobianFactor>(1, I_1x1, Z_1x1,
                                     noiseModel::Isotropic::Sigma(1, 1.0));

  VectorValues values;
  values.insert(1, I_1x1);

  // We are testing the normal distribution PDF where info matrix Σ = 1,
  // mean mu = 0  and x = 1.
  // Therefore factor squared error: y = 0.5 * (Σ*x - mu)^2 =
  // 0.5 * (1.0 - 0)^2 = 0.5
  // NOTE the 0.5 constant is a part of the factor error.
  EXPECT_DOUBLES_EQUAL(0.5, gfg.error(values), 1e-12);

  // The gaussian PDF value is: exp^(-0.5 * (Σ*x - mu)^2) / sqrt(2 * PI)
  // Ignore the denominator and we get: exp^(-0.5 * (1.0)^2) = exp^(-0.5)
  double expected = exp(-0.5);
  EXPECT_DOUBLES_EQUAL(expected, gfg.probPrime(values), 1e-12);
}

TEST(GaussianFactorGraph, InconsistentEliminationMessage) {
  // Create empty graph
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);

  using gtsam::symbol_shorthand::X;
  fg.emplace_shared<JacobianFactor>(0, 10 * I_2x2, -1.0 * Vector::Ones(2),
                                    unit2);
  fg.emplace_shared<JacobianFactor>(0, -10 * I_2x2, 1, 10 * I_2x2,
                                    Vector2(2.0, -1.0), unit2);
  fg.emplace_shared<JacobianFactor>(1, -5 * I_2x2, 2, 5 * I_2x2,
                                    Vector2(-1.0, 1.5), unit2);
  fg.emplace_shared<JacobianFactor>(2, -5 * I_2x2, X(3), 5 * I_2x2,
                                    Vector2(-1.0, 1.5), unit2);

  Ordering ordering{0, 1};

  try {
    fg.eliminateSequential(ordering);
  } catch (const exception& exc) {
    std::string expected_exception_message = "An inference algorithm was called with inconsistent "
        "arguments.  "
        "The\n"
        "factor graph, ordering, or variable index were "
        "inconsistent with "
        "each\n"
        "other, or a full elimination routine was called with "
        "an ordering "
        "that\n"
        "does not include all of the variables.\n"
        "Leftover keys after elimination: 2, x3.";
    EXPECT(expected_exception_message == exc.what());
  }

  // Test large number of keys
  fg = GaussianFactorGraph();
  for (size_t i = 0; i < 1000; i++) {
    fg.emplace_shared<JacobianFactor>(i, -I_2x2, i + 1, I_2x2,
                                      Vector2(2.0, -1.0), unit2);
  }

  try {
    fg.eliminateSequential(ordering);
  } catch (const exception& exc) {
    std::string expected_exception_message = "An inference algorithm was called with inconsistent "
        "arguments.  "
        "The\n"
        "factor graph, ordering, or variable index were "
        "inconsistent with "
        "each\n"
        "other, or a full elimination routine was called with "
        "an ordering "
        "that\n"
        "does not include all of the variables.\n"
        "Leftover keys after elimination: 2, 3, 4, 5, ... (total 999 keys).";
    EXPECT(expected_exception_message == exc.what());
  }
}
/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
