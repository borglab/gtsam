/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactorGraphUnordered.cpp
 *  @brief  Unit tests for Linear Factor
 *  @author Christian Potthast
 *  @author Frank Dellaert
 *  @author Richard Roberts
 **/

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <gtsam/base/debug.h>
#include <gtsam/base/VerticalBlockMatrix.h>
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/GaussianConditional.h>
#include <gtsam/linear/GaussianBayesNet.h>

using namespace std;
using namespace gtsam;

static SharedDiagonal
  sigma0_1 = noiseModel::Isotropic::Sigma(2,0.1), sigma_02 = noiseModel::Isotropic::Sigma(2,0.2),
  constraintModel = noiseModel::Constrained::All(2);

/* ************************************************************************* */
TEST(GaussianFactorGraph, initialization) {
  // Create empty graph
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);

  fg +=
    JacobianFactor(0, 10*eye(2), -1.0*ones(2), unit2),
    JacobianFactor(0, -10*eye(2),1, 10*eye(2), (Vec(2) << 2.0, -1.0), unit2),
    JacobianFactor(0, -5*eye(2), 2, 5*eye(2), (Vec(2) << 0.0, 1.0), unit2),
    JacobianFactor(1, -5*eye(2), 2, 5*eye(2), (Vec(2) << -1.0, 1.5), unit2);

  EXPECT_LONGS_EQUAL(4, (long)fg.size());

  // Test sparse, which takes a vector and returns a matrix, used in MATLAB
  // Note that this the augmented vector and the RHS is in column 7
  Matrix expectedIJS = Matrix_(3,22,
          1.,   2.,  1.,  2.,     3.,   4.,   3.,   4.,  3.,  4.,    5.,  6., 5., 6., 5., 6.,    7.,  8., 7., 8.,  7., 8.,
          1.,   2.,  7.,  7.,     1.,   2.,   3.,   4.,  7.,  7.,    1.,  2., 5., 6., 7., 7.,    3.,  4., 5., 6.,  7., 7.,
          10., 10., -1., -1.,   -10., -10.,  10.,  10.,  2., -1.,   -5., -5., 5., 5., 0., 1.,   -5., -5., 5., 5., -1., 1.5
  );
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

  // Expected - NOTE that we transpose this!
  Matrix expected = Matrix_(16,3,
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
      4., 6.,32.).transpose();

  GaussianFactorGraph gfg;
  SharedDiagonal model = noiseModel::Isotropic::Sigma(2, 0.5);
  gfg.add(0, Matrix_(2,3, 1., 2., 3., 5., 6., 7.), (Vec(2) << 4., 8.), model);
  gfg.add(0, Matrix_(2,3, 9.,10., 0., 0., 0., 0.), 1, Matrix_(2,2, 11., 12., 14., 15.), Vector_(2, 13.,16.), model);

  Matrix actual = gfg.sparseJacobian_();

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, matrices) {
  // Create factor graph:
  // x1 x2 x3 x4 x5  b
  //  1  2  3  0  0  4
  //  5  6  7  0  0  8
  //  9 10  0 11 12 13
  //  0  0  0 14 15 16

  GaussianFactorGraph gfg;
  SharedDiagonal model = noiseModel::Unit::Create(2);
  gfg.add(0, Matrix_(2,3, 1., 2., 3., 5., 6., 7.), (Vec(2) << 4., 8.), model);
  gfg.add(0, Matrix_(2,3, 9.,10., 0., 0., 0., 0.), 1, Matrix_(2,2, 11., 12., 14., 15.), Vector_(2, 13.,16.), model);

  Matrix jacobian(4,6);
  jacobian <<
      1, 2, 3, 0, 0, 4,
      5, 6, 7, 0, 0, 8,
      9,10, 0,11,12,13,
      0, 0, 0,14,15,16;

  Matrix expectedJacobian = jacobian;
  Matrix expectedHessian = jacobian.transpose() * jacobian;
  Matrix expectedA = jacobian.leftCols(jacobian.cols()-1);
  Vector expectedb = jacobian.col(jacobian.cols()-1);
  Matrix expectedL = expectedA.transpose() * expectedA;
  Vector expectedeta = expectedA.transpose() * expectedb;

  Matrix actualJacobian = gfg.augmentedJacobian();
  Matrix actualHessian = gfg.augmentedHessian();
  Matrix actualA; Vector actualb; boost::tie(actualA,actualb) = gfg.jacobian();
  Matrix actualL; Vector actualeta; boost::tie(actualL,actualeta) = gfg.hessian();

  EXPECT(assert_equal(expectedJacobian, actualJacobian));
  EXPECT(assert_equal(expectedHessian, actualHessian));
  EXPECT(assert_equal(expectedA, actualA));
  EXPECT(assert_equal(expectedb, actualb));
  EXPECT(assert_equal(expectedL, actualL));
  EXPECT(assert_equal(expectedeta, actualeta));
}

/* ************************************************************************* */
static GaussianFactorGraph createSimpleGaussianFactorGraph() {
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
  fg += JacobianFactor(2, 10*eye(2), -1.0*ones(2), unit2);
  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg += JacobianFactor(2, -10*eye(2), 0, 10*eye(2), (Vec(2) << 2.0, -1.0), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg += JacobianFactor(2, -5*eye(2), 1, 5*eye(2), (Vec(2) << 0.0, 1.0), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg += JacobianFactor(0, -5*eye(2), 1, 5*eye(2), (Vec(2) << -1.0, 1.5), unit2);
  return fg;
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, gradient )
{
  GaussianFactorGraph fg = createSimpleGaussianFactorGraph();

  // Construct expected gradient
  // 2*f(x) = 100*(x1+c[X(1)])^2 + 100*(x2-x1-[0.2;-0.1])^2 + 25*(l1-x1-[0.0;0.2])^2 + 25*(l1-x2-[-0.2;0.3])^2
  // worked out: df/dx1 = 100*[0.1;0.1] + 100*[0.2;-0.1]) + 25*[0.0;0.2] = [10+20;10-10+5] = [30;5]
  VectorValues expected = map_list_of
    (1, (Vec(2) << 5.0, -12.5))
    (2, (Vec(2) << 30.0, 5.0))
    (0, (Vec(2) << -25.0, 17.5));

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
TEST( GaussianFactorGraph, transposeMultiplication )
{
  GaussianFactorGraph A = createSimpleGaussianFactorGraph();

  Errors e; e +=
    (Vec(2) <<  0.0, 0.0),
    (Vec(2) << 15.0, 0.0),
    (Vec(2) <<  0.0,-5.0),
    (Vec(2) << -7.5,-5.0);

  VectorValues expected;
  expected.insert(1, (Vec(2) <<  -37.5,-50.0));
  expected.insert(2, (Vec(2) << -150.0, 25.0));
  expected.insert(0, (Vec(2) <<  187.5, 25.0));

  VectorValues actual = A.transposeMultiply(e);
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, eliminate_empty )
{
  // eliminate an empty factor
  GaussianFactorGraph gfg;
  gfg.add(JacobianFactor());
  GaussianBayesNet::shared_ptr actualBN;
  GaussianFactorGraph::shared_ptr remainingGFG;
  boost::tie(actualBN, remainingGFG) = gfg.eliminatePartialSequential(Ordering());

  // expected Bayes net is empty
  GaussianBayesNet expectedBN;

  // expected remaining graph should be the same as the original, still containing the empty factor
  GaussianFactorGraph expectedLF = gfg;

  // check if the result matches
  EXPECT(assert_equal(*actualBN, expectedBN));
  EXPECT(assert_equal(*remainingGFG, expectedLF));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, matrices2 )
{
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();
  Matrix A; Vector b; boost::tie(A,b) = gfg.jacobian();
  Matrix AtA; Vector eta; boost::tie(AtA,eta) = gfg.hessian();
  EXPECT(assert_equal(A.transpose()*A, AtA));
  EXPECT(assert_equal(A.transpose()*b, eta));
}


/* ************************************************************************* */
TEST( GaussianFactorGraph, multiplyHessianAdd )
{
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();

  VectorValues x = map_list_of
    (0, (Vec(2) << 1,2))
    (1, (Vec(2) << 3,4))
    (2, (Vec(2) << 5,6));

  VectorValues expected;
  expected.insert(0, (Vec(2) <<  -450, -450));
  expected.insert(1, (Vec(2) << 0, 0));
  expected.insert(2, (Vec(2) <<  950, 1050));

  VectorValues actual;
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(expected, actual));

  // now, do it with non-zero y
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(2*expected, actual));
}

/* ************************************************************************* */
static GaussianFactorGraph createGaussianFactorGraphWithHessianFactor() {
  GaussianFactorGraph gfg = createSimpleGaussianFactorGraph();
#ifdef LUCA
  gfg += HessianFactor(1, 2, 100*ones(2,2), 200*ones(2,2), (Vec(2) << 0.0, 1.0),
                                            400*ones(2,2), (Vec(2) << 1.0, 1.0), 3.0);
#else
  gfg += HessianFactor(1, 2, 100*eye(2,2), zeros(2,2),   (Vec(2) << 0.0, 1.0),
                                           400*eye(2,2), (Vec(2) << 1.0, 1.0), 3.0);
#endif
  return gfg;
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, multiplyHessianAdd2 )
{
  GaussianFactorGraph gfg = createGaussianFactorGraphWithHessianFactor();

  VectorValues x = map_list_of
    (0, (Vec(2) << 1,2))
    (1, (Vec(2) << 3,4))
    (2, (Vec(2) << 5,6));

  VectorValues expected;
#ifdef LUCA
  // expected from matlab: -450        -450        2900        2900        6750        6850
  expected.insert(0, (Vec(2) <<  -450, -450));
  expected.insert(1, (Vec(2) << 2900, 2900));
  expected.insert(2, (Vec(2) <<  6750, 6850));
#else
  expected.insert(0, (Vec(2) <<  -450, -450));
  expected.insert(1, (Vec(2) <<  300, 400));
  expected.insert(2, (Vec(2) << 2950, 3450));
#endif

  VectorValues actual;
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(expected, actual));

  // now, do it with non-zero y
  gfg.multiplyHessianAdd(1.0, x, actual);
  EXPECT(assert_equal(2*expected, actual));
}


/* ************************************************************************* */
TEST( GaussianFactorGraph, matricesMixed )
{
  GaussianFactorGraph gfg = createGaussianFactorGraphWithHessianFactor();
  Matrix A; Vector b; boost::tie(A,b) = gfg.jacobian(); // incorrect !
  Matrix AtA; Vector eta; boost::tie(AtA,eta) = gfg.hessian(); // correct
  EXPECT(assert_equal(A.transpose()*A, AtA));
  Vector expected = - (Vec(6) << -25, 17.5, 5, -13.5, 29, 4);
  EXPECT(assert_equal(expected, eta));
  EXPECT(assert_equal(A.transpose()*b, eta));
}


/* ************************************************************************* */
TEST( GaussianFactorGraph, gradientAtZero )
{
  GaussianFactorGraph gfg = createGaussianFactorGraphWithHessianFactor();
  VectorValues expected;
  VectorValues actual = gfg.gradientAtZero();
  expected.insert(0, (Vec(2) << -25, 17.5));
  expected.insert(1, (Vec(2) << 5, -13.5));
  expected.insert(2, (Vec(2) << 29, 4));
  EXPECT(assert_equal(expected, actual));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
