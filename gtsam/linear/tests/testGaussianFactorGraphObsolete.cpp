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
#include <gtsam/inference/VariableSlots.h>
#include <gtsam/inference/VariableIndexOrdered.h>
#include <gtsam/linear/GaussianFactorGraphOrdered.h>
#include <gtsam/linear/GaussianSequentialSolver.h>

using namespace std;
using namespace gtsam;
using namespace boost;

static SharedDiagonal
  sigma0_1 = noiseModel::Isotropic::Sigma(2,0.1), sigma_02 = noiseModel::Isotropic::Sigma(2,0.2),
  constraintModel = noiseModel::Constrained::All(2);

static GaussianFactorGraphOrdered createSimpleGaussianFactorGraph() {
  GaussianFactorGraphOrdered fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);
  // linearized prior on x1: c[_x1_]+x1=0 i.e. x1=-c[_x1_]
  fg.add(2, 10*eye(2), -1.0*ones(2), unit2);
  // odometry between x1 and x2: x2-x1=[0.2;-0.1]
  fg.add(2, -10*eye(2), 0, 10*eye(2), Vector_(2, 2.0, -1.0), unit2);
  // measurement between x1 and l1: l1-x1=[0.0;0.2]
  fg.add(2, -5*eye(2), 1, 5*eye(2), Vector_(2, 0.0, 1.0), unit2);
  // measurement between x2 and l1: l1-x2=[-0.2;0.3]
  fg.add(0, -5*eye(2), 1, 5*eye(2), Vector_(2, -1.0, 1.5), unit2);
  return fg;
}


/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, initialization) {
  // Create empty graph
  GaussianFactorGraphOrdered fg;
  SharedDiagonal unit2 = noiseModel::Unit::Create(2);

  fg.add(0, 10*eye(2), -1.0*ones(2), unit2);
  fg.add(0, -10*eye(2),1, 10*eye(2), Vector_(2, 2.0, -1.0), unit2);
  fg.add(0, -5*eye(2), 2, 5*eye(2), Vector_(2, 0.0, 1.0), unit2);
  fg.add(1, -5*eye(2), 2, 5*eye(2), Vector_(2, -1.0, 1.5), unit2);

  EXPECT_LONGS_EQUAL(4, fg.size());
  JacobianFactorOrdered factor = *boost::dynamic_pointer_cast<JacobianFactorOrdered>(fg[0]);

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
TEST(GaussianFactorGraphOrdered, CombineJacobians)
{
  Matrix A01 = Matrix_(3,3,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0);
  Vector b0 = Vector_(3, 1.5, 1.5, 1.5);
  Vector s0 = Vector_(3, 1.6, 1.6, 1.6);

  Matrix A10 = Matrix_(3,3,
      2.0, 0.0, 0.0,
      0.0, 2.0, 0.0,
      0.0, 0.0, 2.0);
  Matrix A11 = Matrix_(3,3,
      -2.0, 0.0, 0.0,
      0.0, -2.0, 0.0,
      0.0, 0.0, -2.0);
  Vector b1 = Vector_(3, 2.5, 2.5, 2.5);
  Vector s1 = Vector_(3, 2.6, 2.6, 2.6);

  Matrix A21 = Matrix_(3,3,
      3.0, 0.0, 0.0,
      0.0, 3.0, 0.0,
      0.0, 0.0, 3.0);
  Vector b2 = Vector_(3, 3.5, 3.5, 3.5);
  Vector s2 = Vector_(3, 3.6, 3.6, 3.6);

  GaussianFactorGraphOrdered gfg;
  gfg.add(1, A01, b0, noiseModel::Diagonal::Sigmas(s0, true));
  gfg.add(0, A10, 1, A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
  gfg.add(1, A21, b2, noiseModel::Diagonal::Sigmas(s2, true));

  // Convert to Jacobians (inefficient copy of all factors instead of selectively converting only Hessians)
  FactorGraphOrdered<JacobianFactorOrdered> jacobians;
  BOOST_FOREACH(const GaussianFactorGraphOrdered::sharedFactor& factor, gfg) {
    jacobians.push_back(boost::make_shared<JacobianFactorOrdered>(*factor));
  }

  // Combine Jacobians into a single dense factor
  JacobianFactorOrdered actual = *CombineJacobians(jacobians, VariableSlots(gfg));

  Matrix zero3x3 = zeros(3,3);
  Matrix A0 = gtsam::stack(3, &zero3x3, &A10, &zero3x3);
  Matrix A1 = gtsam::stack(3, &A01, &A11, &A21);
  Vector b = gtsam::concatVectors(3, &b0, &b1, &b2);
  Vector sigmas = gtsam::concatVectors(3, &s0, &s1, &s2);

  JacobianFactorOrdered expected(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorOrdered, CombineAndEliminate)
{
  Matrix A01 = Matrix_(3,3,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0);
  Vector b0 = Vector_(3, 1.5, 1.5, 1.5);
  Vector s0 = Vector_(3, 1.6, 1.6, 1.6);

  Matrix A10 = Matrix_(3,3,
      2.0, 0.0, 0.0,
      0.0, 2.0, 0.0,
      0.0, 0.0, 2.0);
  Matrix A11 = Matrix_(3,3,
      -2.0, 0.0, 0.0,
      0.0, -2.0, 0.0,
      0.0, 0.0, -2.0);
  Vector b1 = Vector_(3, 2.5, 2.5, 2.5);
  Vector s1 = Vector_(3, 2.6, 2.6, 2.6);

  Matrix A21 = Matrix_(3,3,
      3.0, 0.0, 0.0,
      0.0, 3.0, 0.0,
      0.0, 0.0, 3.0);
  Vector b2 = Vector_(3, 3.5, 3.5, 3.5);
  Vector s2 = Vector_(3, 3.6, 3.6, 3.6);

  GaussianFactorGraphOrdered gfg;
  gfg.add(1, A01, b0, noiseModel::Diagonal::Sigmas(s0, true));
  gfg.add(0, A10, 1, A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
  gfg.add(1, A21, b2, noiseModel::Diagonal::Sigmas(s2, true));

  Matrix zero3x3 = zeros(3,3);
  Matrix A0 = gtsam::stack(3, &A10, &zero3x3, &zero3x3);
  Matrix A1 = gtsam::stack(3, &A11, &A01, &A21);
  Vector b = gtsam::concatVectors(3, &b1, &b0, &b2);
  Vector sigmas = gtsam::concatVectors(3, &s1, &s0, &s2);

  JacobianFactorOrdered expectedFactor(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));
  GaussianConditionalOrdered::shared_ptr expectedBN = expectedFactor.eliminate(1);

  GaussianConditionalOrdered::shared_ptr actualBN;
  GaussianFactorOrdered::shared_ptr actualFactor;
  boost::tie(actualBN, actualFactor) = EliminateQROrdered(gfg, 1);
  JacobianFactorOrdered::shared_ptr actualJacobian = boost::dynamic_pointer_cast<
      JacobianFactorOrdered>(actualFactor);

  EXPECT(assert_equal(*expectedBN, *actualBN));
  EXPECT(assert_equal(expectedFactor, *actualJacobian));
}

/* ************************************************************************* */
TEST(GaussianFactorOrdered, eliminateFrontals)
{
  // Augmented Ab test case for whole factor graph
  Matrix Ab = Matrix_(14,11,
      4.,     0.,     1.,     4.,     1.,     0.,     3.,     6.,     8.,     8.,     1.,
      9.,     2.,     0.,     1.,     6.,     3.,     9.,     6.,     6.,     9.,     4.,
      5.,     3.,     7.,     9.,     5.,     5.,     9.,     1.,     3.,     7.,     0.,
      5.,     6.,     5.,     7.,     9.,     4.,     0.,     1.,     1.,     3.,     5.,
      0.,     0.,     4.,     5.,     6.,     6.,     7.,     9.,     4.,     5.,     4.,
      0.,     0.,     9.,     4.,     8.,     6.,     2.,     1.,     4.,     1.,     6.,
      0.,     0.,     6.,     0.,     4.,     2.,     4.,     0.,     1.,     9.,     6.,
      0.,     0.,     6.,     6.,     4.,     4.,     5.,     5.,     5.,     8.,     6.,
      0.,     0.,     0.,     0.,     8.,     0.,     9.,     8.,     2.,     8.,     0.,
      0.,     0.,     0.,     0.,     0.,     9.,     4.,     6.,     3.,     2.,     0.,
      0.,     0.,     0.,     0.,     1.,     1.,     9.,     1.,     5.,     5.,     3.,
      0.,     0.,     0.,     0.,     1.,     1.,     3.,     3.,     2.,     0.,     5.,
      0.,     0.,     0.,     0.,     0.,     0.,     0.,     0.,     2.,     4.,     6.,
      0.,     0.,     0.,     0.,     0.,     0.,     0.,     0.,     6.,     3.,     4.);

  // Create first factor (from pieces of Ab)
  list<pair<Index, Matrix> > terms1;

  terms1 +=
      make_pair( 3, Matrix(Ab.block(0, 0, 4, 2))),
      make_pair( 5, Matrix(Ab.block(0, 2, 4, 2))),
      make_pair( 7, Matrix(Ab.block(0, 4, 4, 2))),
      make_pair( 9, Matrix(Ab.block(0, 6, 4, 2))),
      make_pair(11, Matrix(Ab.block(0, 8, 4, 2)));
  Vector b1 = Ab.col(10).segment(0, 4);
  JacobianFactorOrdered::shared_ptr factor1(new JacobianFactorOrdered(terms1, b1, noiseModel::Isotropic::Sigma(4, 0.5)));

  // Create second factor
  list<pair<Index, Matrix> > terms2;
  terms2 +=
      make_pair(5, Matrix(Ab.block(4, 2, 4, 2))),
      make_pair(7, Matrix(Ab.block(4, 4, 4, 2))),
      make_pair(9, Matrix(Ab.block(4, 6, 4, 2))),
      make_pair(11,Matrix(Ab.block(4, 8, 4, 2)));
  Vector b2 = Ab.col(10).segment(4, 4);
  JacobianFactorOrdered::shared_ptr factor2(new JacobianFactorOrdered(terms2, b2, noiseModel::Isotropic::Sigma(4, 0.5)));

  // Create third factor
  list<pair<Index, Matrix> > terms3;
  terms3 +=
      make_pair(7, Matrix(Ab.block(8, 4, 4, 2))),
      make_pair(9, Matrix(Ab.block(8, 6, 4, 2))),
      make_pair(11,Matrix(Ab.block(8, 8, 4, 2)));
  Vector b3 = Ab.col(10).segment(8, 4);
  JacobianFactorOrdered::shared_ptr factor3(new JacobianFactorOrdered(terms3, b3, noiseModel::Isotropic::Sigma(4, 0.5)));

  // Create fourth factor
  list<pair<Index, Matrix> > terms4;
  terms4 +=
      make_pair(11, Matrix(Ab.block(12, 8, 2, 2)));
  Vector b4 = Ab.col(10).segment(12, 2);
  JacobianFactorOrdered::shared_ptr factor4(new JacobianFactorOrdered(terms4, b4, noiseModel::Isotropic::Sigma(2, 0.5)));

  // Create factor graph
  GaussianFactorGraphOrdered factors;
  factors.push_back(factor1);
  factors.push_back(factor2);
  factors.push_back(factor3);
  factors.push_back(factor4);

  // extract the dense matrix for the graph
  Matrix actualDense = factors.augmentedJacobian();
  EXPECT(assert_equal(2.0 * Ab, actualDense));

  // Convert to Jacobians, inefficient copy of all factors instead of selectively converting only Hessians
  FactorGraphOrdered<JacobianFactorOrdered> jacobians;
  BOOST_FOREACH(const GaussianFactorGraphOrdered::sharedFactor& factor, factors) {
    jacobians.push_back(boost::make_shared<JacobianFactorOrdered>(*factor));
  }

  // Create combined factor
  JacobianFactorOrdered combined(*CombineJacobians(jacobians, VariableSlots(factors)));

  // Copies factors as they will be eliminated in place
  JacobianFactorOrdered actualFactor_QR = combined;
  JacobianFactorOrdered actualFactor_Chol = combined;

  // Expected augmented matrix, both GaussianConditional (first 6 rows) and remaining factor (next 4 rows)
  Matrix R = 2.0*Matrix_(11,11,
      -12.1244,  -5.1962,  -5.2786,  -8.6603, -10.5573,  -5.9385, -11.3820,  -7.2581,  -8.7427, -13.4440,  -5.3611,
            0.,   4.6904,   5.0254,   5.5432,   5.5737,   3.0153,  -3.0153,  -3.5635,  -3.9290,  -2.7412,   2.1625,
            0.,       0., -13.8160,  -8.7166, -10.2245,  -8.8666,  -8.7632,  -5.2544,  -6.9192, -10.5537,  -9.3250,
            0.,       0.,       0.,   6.5033,  -1.1453,   1.3179,   2.5768,   5.5503,   3.6524,   1.3491,  -2.5676,
            0.,       0.,       0.,       0.,  -9.6242,  -2.1148,  -9.3509, -10.5846,  -3.5366,  -6.8561,  -3.2277,
            0.,       0.,       0.,       0.,       0.,   9.7887,   4.3551,   5.7572,   2.7876,   0.1611,   1.1769,
            0.,       0.,       0.,       0.,       0.,       0., -11.1139,  -0.6521,  -2.1943,  -7.5529,  -0.9081,
            0.,       0.,       0.,       0.,       0.,       0.,       0.,  -4.6479,  -1.9367,  -6.5170,  -3.7685,
            0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,   8.2503,   3.3757,   6.8476,
            0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,  -5.7095,  -0.0090,
            0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,       0.,  -7.1635);

  // Expected conditional on first variable from first 2 rows of R
  Matrix R1 = sub(R, 0,2, 0,2);
  list<pair<Index, Matrix> > cterms1;
  cterms1 +=
      make_pair(5, sub(R, 0,2, 2,4 )),
      make_pair(7, sub(R, 0,2, 4,6 )),
      make_pair(9, sub(R, 0,2, 6,8 )),
      make_pair(11,sub(R, 0,2, 8,10));
  Vector d1 = R.col(10).segment(0,2);
  GaussianConditionalOrdered::shared_ptr cond1(new GaussianConditionalOrdered(3, d1, R1, cterms1, ones(2)));

  // Expected conditional on second variable from next 2 rows of R
  Matrix R2 = sub(R, 2,4, 2,4);
  list<pair<Index, Matrix> > cterms2;
  cterms2 +=
      make_pair(7, sub(R, 2,4, 4,6)),
      make_pair(9, sub(R, 2,4, 6,8)),
      make_pair(11,sub(R, 2,4, 8,10));
  Vector d2 = R.col(10).segment(2,2);
  GaussianConditionalOrdered::shared_ptr cond2(new GaussianConditionalOrdered(5, d2, R2, cterms2, ones(2)));

  // Expected conditional on third variable from next 2 rows of R
  Matrix R3 = sub(R, 4,6, 4,6);
  list<pair<Index, Matrix> > cterms3;
  cterms3 +=
      make_pair(9,  sub(R, 4,6, 6,8)),
      make_pair(11, sub(R, 4,6, 8,10));
  Vector d3 = R.col(10).segment(4,2);
  GaussianConditionalOrdered::shared_ptr cond3(new GaussianConditionalOrdered(7, d3, R3, cterms3, ones(2)));

  // Create expected Bayes net fragment from three conditionals above
//  GaussianBayesNetOrdered expectedFragment;
//  expectedFragment.push_back(cond1);
//  expectedFragment.push_back(cond2);
//  expectedFragment.push_back(cond3);
  Index ikeys[] = {3,5,7,9,11};
  std::vector<Index> keys(ikeys, ikeys + sizeof(ikeys)/sizeof(Index));
  size_t dims[] = { 2,2,2,2,2,1 };
  size_t height = 11;
  VerticalBlockView<Matrix> Rblock(R, dims, dims+6, height);
  GaussianConditionalOrdered::shared_ptr expectedFragment( new GaussianConditionalOrdered(keys.begin(), keys.end(), 3,
      Rblock, ones(6)) );

  // Get expected matrices for remaining factor
  Matrix Ae1 = sub(R, 6,10, 6,8);
  Matrix Ae2 = sub(R, 6,10, 8,10);
  Vector be = R.col(10).segment(6, 4);

  // Eliminate (3 frontal variables, 6 scalar columns) using QR !!!!
  GaussianConditionalOrdered::shared_ptr actualFragment_QR = actualFactor_QR.eliminate(3);

  EXPECT(assert_equal(*expectedFragment, *actualFragment_QR, 0.001));
  EXPECT(assert_equal(size_t(2), actualFactor_QR.keys().size()));
  EXPECT(assert_equal(Index(9), actualFactor_QR.keys()[0]));
  EXPECT(assert_equal(Index(11), actualFactor_QR.keys()[1]));
  EXPECT(assert_equal(Ae1, actualFactor_QR.getA(actualFactor_QR.begin()), 0.001));
  EXPECT(assert_equal(Ae2, actualFactor_QR.getA(actualFactor_QR.begin()+1), 0.001));
  EXPECT(assert_equal(be, actualFactor_QR.getb(), 0.001));
  EXPECT(assert_equal(ones(4), actualFactor_QR.get_model()->sigmas(), 0.001));

  // Eliminate (3 frontal variables, 6 scalar columns) using Cholesky !!!!
#ifdef BROKEN
  GaussianBayesNetOrdered actualFragment_Chol = *actualFactor_Chol.eliminate(3, JacobianFactorOrdered::SOLVE_CHOLESKY);
  EXPECT(assert_equal(expectedFragment, actualFragment_Chol, 0.001));
  EXPECT(assert_equal(size_t(2), actualFactor_Chol.keys().size()));
  EXPECT(assert_equal(Index(9), actualFactor_Chol.keys()[0]));
  EXPECT(assert_equal(Index(11), actualFactor_Chol.keys()[1]));
  EXPECT(assert_equal(Ae1, actualFactor_Chol.getA(actualFactor_Chol.begin()), 0.001)); ////
  EXPECT(linear_dependent(Ae2, actualFactor_Chol.getA(actualFactor_Chol.begin()+1), 0.001));
  EXPECT(assert_equal(be, actualFactor_Chol.getb(), 0.001)); ////
  EXPECT(assert_equal(ones(4), actualFactor_Chol.get_sigmas(), 0.001));
#endif
}

/* ************************************************************************* */
TEST(GaussianFactorOrdered, permuteWithInverse)
{
  Matrix A1 = Matrix_(2,2,
      1.0, 2.0,
      3.0, 4.0);
  Matrix A2 = Matrix_(2,1,
      5.0,
      6.0);
  Matrix A3 = Matrix_(2,3,
      7.0, 8.0, 9.0,
      10.0, 11.0, 12.0);
  Vector b = Vector_(2, 13.0, 14.0);

  Permutation inversePermutation(6);
  inversePermutation[0] = 5;
  inversePermutation[1] = 4;
  inversePermutation[2] = 3;
  inversePermutation[3] = 2;
  inversePermutation[4] = 1;
  inversePermutation[5] = 0;

  JacobianFactorOrdered actual(1, A1, 3, A2, 5, A3, b, noiseModel::Isotropic::Sigma(2, 1.0));
  GaussianFactorGraphOrdered actualFG; actualFG.push_back(JacobianFactorOrdered::shared_ptr(new JacobianFactorOrdered(actual)));
  VariableIndexOrdered actualIndex(actualFG);
  actual.permuteWithInverse(inversePermutation);
//  actualIndex.permute(*inversePermutation.inverse());

  JacobianFactorOrdered expected(4, A1, 2, A2, 0, A3, b, noiseModel::Isotropic::Sigma(2, 1.0));
  GaussianFactorGraphOrdered expectedFG; expectedFG.push_back(JacobianFactorOrdered::shared_ptr(new JacobianFactorOrdered(expected)));
//  GaussianVariableIndex expectedIndex(expectedFG);

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, sparseJacobian) {
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

  GaussianFactorGraphOrdered gfg;
  SharedDiagonal model = noiseModel::Isotropic::Sigma(2, 0.5);
  gfg.add(0, Matrix_(2,3, 1., 2., 3., 5., 6., 7.), Vector_(2, 4., 8.), model);
  gfg.add(0, Matrix_(2,3, 9.,10., 0., 0., 0., 0.), 1, Matrix_(2,2, 11., 12., 14., 15.), Vector_(2, 13.,16.), model);

  Matrix actual = gfg.sparseJacobian_();

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, matrices) {
  // Create factor graph:
  // x1 x2 x3 x4 x5  b
  //  1  2  3  0  0  4
  //  5  6  7  0  0  8
  //  9 10  0 11 12 13
  //  0  0  0 14 15 16

  GaussianFactorGraphOrdered gfg;
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
TEST( GaussianFactorGraphOrdered, gradient )
{
  GaussianFactorGraphOrdered fg = createSimpleGaussianFactorGraph();

  // Construct expected gradient
  VectorValuesOrdered expected;

  // 2*f(x) = 100*(x1+c[X(1)])^2 + 100*(x2-x1-[0.2;-0.1])^2 + 25*(l1-x1-[0.0;0.2])^2 + 25*(l1-x2-[-0.2;0.3])^2
  // worked out: df/dx1 = 100*[0.1;0.1] + 100*[0.2;-0.1]) + 25*[0.0;0.2] = [10+20;10-10+5] = [30;5]
  expected.insert(1,Vector_(2,  5.0,-12.5));
  expected.insert(2,Vector_(2, 30.0,  5.0));
  expected.insert(0,Vector_(2,-25.0, 17.5));

  // Check the gradient at delta=0
  VectorValuesOrdered zero = VectorValuesOrdered::Zero(expected);
  VectorValuesOrdered actual = gradient(fg, zero);
  EXPECT(assert_equal(expected,actual));

  // Check the gradient at the solution (should be zero)
  VectorValuesOrdered solution = *GaussianSequentialSolver(fg).optimize();
  VectorValuesOrdered actual2 = gradient(fg, solution);
  EXPECT(assert_equal(VectorValuesOrdered::Zero(solution), actual2));
}

/* ************************************************************************* */
TEST( GaussianFactorGraphOrdered, transposeMultiplication )
{
  GaussianFactorGraphOrdered A = createSimpleGaussianFactorGraph();

  VectorValuesOrdered e;
  e.insert(0, Vector_(2, 0.0, 0.0));
  e.insert(1, Vector_(2,15.0, 0.0));
  e.insert(2, Vector_(2, 0.0,-5.0));
  e.insert(3, Vector_(2,-7.5,-5.0));

  VectorValuesOrdered expected;
  expected.insert(1, Vector_(2, -37.5,-50.0));
  expected.insert(2, Vector_(2,-150.0, 25.0));
  expected.insert(0, Vector_(2, 187.5, 25.0));

  VectorValuesOrdered actual = VectorValuesOrdered::SameStructure(expected);
  transposeMultiply(A, e, actual);
  EXPECT(assert_equal(expected,actual));
}

/* ************************************************************************* */
TEST(GaussianFactorGraphOrdered, eliminate_empty )
{
  // eliminate an empty factor
  GaussianFactorGraphOrdered gfg;
  gfg.push_back(boost::make_shared<JacobianFactorOrdered>());
  GaussianConditionalOrdered::shared_ptr actualCG;
  GaussianFactorGraphOrdered remainingGFG;
  boost::tie(actualCG, remainingGFG) = gfg.eliminateOne(0);

  // expected Conditional Gaussian is just a parent-less node with P(x)=1
  GaussianConditionalOrdered expectedCG(0, Vector(), Matrix(), Vector());

  // expected remaining graph should be the same as the original, still empty :-)
  GaussianFactorGraphOrdered expectedLF = gfg;

  // check if the result matches
  EXPECT(actualCG->equals(expectedCG));
  EXPECT(remainingGFG.equals(expectedLF));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
