/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file   testGaussianFactor.cpp
 *  @brief  Unit tests for Linear Factor
 *  @author Christian Potthast
 *  @author Frank Dellaert
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

static GaussianFactorGraph createSimpleGaussianFactorGraph() {
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
  fg += JacobianFactor(2, 10*eye(2), -1.0*ones(2), unit2);
  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg += JacobianFactor(2, -10*eye(2), 0, 10*eye(2), Vector_(2, 2.0, -1.0), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg += JacobianFactor(2, -5*eye(2), 1, 5*eye(2), Vector_(2, 0.0, 1.0), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg += JacobianFactor(0, -5*eye(2), 1, 5*eye(2), Vector_(2, -1.0, 1.5), unit2);
  return fg;
}

/* ************************************************************************* */
TEST(GaussianFactorGraph, initialization) {
  // Create empty graph
  GaussianFactorGraph fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);

  fg +=
    JacobianFactor(0, 10*eye(2), -1.0*ones(2), unit2),
    JacobianFactor(0, -10*eye(2),1, 10*eye(2), Vector_(2, 2.0, -1.0), unit2),
    JacobianFactor(0, -5*eye(2), 2, 5*eye(2), Vector_(2, 0.0, 1.0), unit2),
    JacobianFactor(1, -5*eye(2), 2, 5*eye(2), Vector_(2, -1.0, 1.5), unit2);

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
  gfg.add(0, Matrix_(2,3, 1., 2., 3., 5., 6., 7.), Vector_(2, 4., 8.), model);
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
  gfg.add(0, Matrix_(2,3, 1., 2., 3., 5., 6., 7.), Vector_(2, 4., 8.), model);
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
  //Matrix actualHessian = gfg.augmentedHessian();
  Matrix actualA; Vector actualb; boost::tie(actualA,actualb) = gfg.jacobian();
  //Matrix actualL; Vector actualeta; boost::tie(actualL,actualeta) = gfg.hessian();

  EXPECT(assert_equal(expectedJacobian, actualJacobian));
  //EXPECT(assert_equal(expectedHessian, actualHessian));
  EXPECT(assert_equal(expectedA, actualA));
  EXPECT(assert_equal(expectedb, actualb));
  //EXPECT(assert_equal(expectedL, actualL));
  //EXPECT(assert_equal(expectedeta, actualeta));
}

/* ************************************************************************* */
TEST( GaussianFactorGraph, gradient )
{
  GaussianFactorGraph fg = createSimpleGaussianFactorGraph();

  // Construct expected gradient
  // 2*f(x) = 100*(x1+c[X(1)])^2 + 100*(x2-x1-[0.2;-0.1])^2 + 25*(l1-x1-[0.0;0.2])^2 + 25*(l1-x2-[-0.2;0.3])^2
  // worked out: df/dx1 = 100*[0.1;0.1] + 100*[0.2;-0.1]) + 25*[0.0;0.2] = [10+20;10-10+5] = [30;5]
  VectorValues expected = map_list_of
    (1, Vector_(2,  5.0,-12.5))
    (2, Vector_(2, 30.0,  5.0))
    (0, Vector_(2,-25.0, 17.5));

  // Check the gradient at delta=0
  VectorValues zero = VectorValues::Zero(expected);
  VectorValues actual = fg.gradient(zero);
  EXPECT(assert_equal(expected, actual));

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
    Vector_(2, 0.0, 0.0),
    Vector_(2,15.0, 0.0),
    Vector_(2, 0.0,-5.0),
    Vector_(2,-7.5,-5.0);

  VectorValues expected;
  expected.insert(1, Vector_(2, -37.5,-50.0));
  expected.insert(2, Vector_(2,-150.0, 25.0));
  expected.insert(0, Vector_(2, 187.5, 25.0));

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
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
