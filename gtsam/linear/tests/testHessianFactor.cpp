/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCholeskyFactor.cpp
 * @brief   
 * @author  Richard Roberts
 * @created Dec 15, 2010
 */

#include <gtsam/base/debug.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <vector>
#include <utility>

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace gtsam;
using namespace std;

/* ************************************************************************* */
TEST(HessianFactor, ConversionConstructor) {

  HessianFactor expected;
  expected.keys_.push_back(0);
  expected.keys_.push_back(1);
  size_t dims[] = { 2, 4, 1 };
  expected.info_.resize(dims, dims+3, false);
  expected.matrix_ = Matrix_(7,7,
                             125.0000,       0.0,  -25.0000,       0.0, -100.0000,       0.0,   25.0000,
                                  0.0,  125.0000,       0.0,  -25.0000,       0.0, -100.0000,  -17.5000,
                             -25.0000,       0.0,   25.0000,       0.0,       0.0,       0.0,   -5.0000,
                                  0.0,  -25.0000,       0.0,   25.0000,       0.0,       0.0,    7.5000,
                            -100.0000,       0.0,       0.0,       0.0,  100.0000,       0.0,  -20.0000,
                                  0.0, -100.0000,       0.0,       0.0,       0.0,  100.0000,   10.0000,
                              25.0000,  -17.5000,   -5.0000,    7.5000,  -20.0000,   10.0000,    8.2500);

  // sigmas
  double sigma1 = 0.2;
  double sigma2 = 0.1;
  Vector sigmas = Vector_(4, sigma1, sigma1, sigma2, sigma2);

  // the combined linear factor
  Matrix Ax2 = Matrix_(4,2,
      // x2
      -1., 0.,
      +0.,-1.,
      1., 0.,
      +0.,1.
  );

  Matrix Al1x1 = Matrix_(4,4,
      // l1   x1
      1., 0., 0.00,  0., // f4
      0., 1., 0.00,  0., // f4
      0., 0., -1.,  0., // f2
      0., 0., 0.00,-1.  // f2
  );

  // the RHS
  Vector b2(4);
  b2(0) = -0.2;
  b2(1) =  0.3;
  b2(2) =  0.2;
  b2(3) = -0.1;

  vector<pair<Index, Matrix> > meas;
  meas.push_back(make_pair(0, Ax2));
  meas.push_back(make_pair(1, Al1x1));
  JacobianFactor combined(meas, b2, sigmas);

  HessianFactor actual(combined);

  VectorValues values(std::vector<size_t>(dims, dims+2));
  values[0] = Vector_(2, 1.0, 2.0);
  values[1] = Vector_(4, 3.0, 4.0, 5.0, 6.0);

  DOUBLES_EQUAL(combined.error(values), actual.error(values), 1e-9);
  EXPECT(assert_equal(expected, actual, 1e-9));
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor1)
{
  Matrix G = Matrix_(2,2, 3.0, 5.0, 0.0, 6.0);
  Vector g = Vector_(2, -8.0, -9.0);
  double f = 10.0;

  Vector dxv = Vector_(2, 1.5, 2.5);

  vector<size_t> dims;
  dims.push_back(2);
  VectorValues dx(dims);

  dx[0] = dxv;

  HessianFactor factor(0, G, g, f);

  double expected = 80.375;
  double actual = factor.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor2)
{
  Matrix G11 = Matrix_(1,1, 1.0);
  Matrix G12 = Matrix_(1,2, 2.0, 4.0);
  Matrix G22 = Matrix_(2,2, 3.0, 5.0, 0.0, 6.0);
  Vector g1 = Vector_(1, -7.0);
  Vector g2 = Vector_(2, -8.0, -9.0);
  double f = 10.0;

  Vector dx0 = Vector_(1, 0.5);
  Vector dx1 = Vector_(2, 1.5, 2.5);

  vector<size_t> dims;
  dims.push_back(1);
  dims.push_back(2);
  VectorValues dx(dims);

  dx[0] = dx0;
  dx[1] = dx1;

  HessianFactor factor(0, 1, G11, G12, g1, G22, g2, f);

  double expected = 90.5;
  double actual = factor.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
}

/* ************************************************************************* */
TEST(HessianFactor, CombineAndEliminate)
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

  GaussianFactorGraph gfg;
  gfg.add(1, A01, b0, noiseModel::Diagonal::Sigmas(s0, true));
  gfg.add(0, A10, 1, A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
  gfg.add(1, A21, b2, noiseModel::Diagonal::Sigmas(s2, true));

  Matrix zero3x3 = zeros(3,3);
  Matrix A0 = gtsam::stack(3, &A10, &zero3x3, &zero3x3);
  Matrix A1 = gtsam::stack(3, &A11, &A01, &A21);
  Vector b = gtsam::concatVectors(3, &b1, &b0, &b2);
  Vector sigmas = gtsam::concatVectors(3, &s1, &s0, &s2);

  JacobianFactor expectedFactor(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));
  GaussianBayesNet expectedBN(*expectedFactor.eliminate(1));

  GaussianFactorGraph::EliminationResult actualCholesky = EliminateCholesky(
			*gfg.convertCastFactors<FactorGraph<HessianFactor> > (), 1);
	HessianFactor::shared_ptr actualFactor = boost::dynamic_pointer_cast<
			HessianFactor>(actualCholesky.second);

  EXPECT(assert_equal(expectedBN, *actualCholesky.first, 1e-6));
  EXPECT(assert_equal(HessianFactor(expectedFactor), *actualFactor, 1e-6));
}

/* ************************************************************************* */
TEST(HessianFactor, eliminate2 )
{
  // sigmas
  double sigma1 = 0.2;
  double sigma2 = 0.1;
  Vector sigmas = Vector_(4, sigma1, sigma1, sigma2, sigma2);

  // the combined linear factor
  Matrix Ax2 = Matrix_(4,2,
      // x2
      -1., 0.,
      +0.,-1.,
      1., 0.,
      +0.,1.
  );

  Matrix Al1x1 = Matrix_(4,4,
      // l1   x1
      1., 0., 0.00,  0., // f4
      0., 1., 0.00,  0., // f4
      0., 0., -1.,  0., // f2
      0., 0., 0.00,-1.  // f2
  );

  // the RHS
  Vector b2(4);
  b2(0) = -0.2;
  b2(1) =  0.3;
  b2(2) =  0.2;
  b2(3) = -0.1;

  vector<pair<Index, Matrix> > meas;
  meas.push_back(make_pair(0, Ax2));
  meas.push_back(make_pair(1, Al1x1));
  JacobianFactor combined(meas, b2, sigmas);

  // eliminate the combined factor
  HessianFactor::shared_ptr combinedLF_Chol(new HessianFactor(combined));
  FactorGraph<HessianFactor> combinedLFG_Chol;
  combinedLFG_Chol.push_back(combinedLF_Chol);

  GaussianFactorGraph::EliminationResult actual_Chol = EliminateCholesky(
			combinedLFG_Chol, 1);
	HessianFactor::shared_ptr actualFactor = boost::dynamic_pointer_cast<
			HessianFactor>(actual_Chol.second);

  // create expected Conditional Gaussian
  double oldSigma = 0.0894427; // from when R was made unit
  Matrix R11 = Matrix_(2,2,
      1.00,  0.00,
      0.00,  1.00
  )/oldSigma;
  Matrix S12 = Matrix_(2,4,
      -0.20, 0.00,-0.80, 0.00,
      +0.00,-0.20,+0.00,-0.80
  )/oldSigma;
  Vector d = Vector_(2,0.2,-0.14)/oldSigma;
  GaussianConditional expectedCG(0,d,R11,1,S12,ones(2));
  EXPECT(assert_equal(expectedCG,*actual_Chol.first->front(),1e-4));

  // the expected linear factor
  double sigma = 0.2236;
  Matrix Bl1x1 = Matrix_(2,4,
      // l1          x1
      1.00, 0.00, -1.00,  0.00,
      0.00, 1.00, +0.00, -1.00
  )/sigma;
  Vector b1 = Vector_(2,0.0,0.894427);
  JacobianFactor expectedLF(1, Bl1x1, b1, repeat(2,1.0));
  EXPECT(assert_equal(HessianFactor(expectedLF), *actualFactor, 1.5e-3));
}

/* ************************************************************************* */
TEST(HessianFactor, eliminateUnsorted) {

  JacobianFactor::shared_ptr factor1(
      new JacobianFactor(0,
                         Matrix_(3,3,
                                 44.7214,     0.0,       0.0,
                                 0.0,     44.7214,       0.0,
                                 0.0,         0.0,   44.7214),
                         1,
                         Matrix_(3,3,
                                 -0.179168,    -44.721,  0.717294,
                                 44.721, -0.179168,  -44.9138,
                                 0.0,         0.0,  -44.7214),
                         Vector_(3, 1.98916e-17, -4.96503e-15, -7.75792e-17),
                         noiseModel::Unit::Create(3)));
  HessianFactor::shared_ptr unsorted_factor2(
      new HessianFactor(JacobianFactor(0,
                        Matrix_(6,3,
                                25.8367,    0.1211,    0.0593,
                                    0.0,   23.4099,   30.8733,
                                    0.0,       0.0,   25.8729,
                                    0.0,       0.0,       0.0,
                                    0.0,       0.0,       0.0,
                                    0.0,       0.0,       0.0),
                        1,
                        Matrix_(6,3,
                                25.7429,   -1.6897,    0.4587,
                                 1.6400,   23.3095,   -8.4816,
                                 0.0034,    0.0509,  -25.7855,
                                 0.9997,   -0.0002,    0.0824,
                                    0.0,    0.9973,    0.9517,
                                    0.0,       0.0,    0.9973),
                        Vector_(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        noiseModel::Unit::Create(6))));
  Permutation permutation(2);
  permutation[0] = 1;
  permutation[1] = 0;
  unsorted_factor2->permuteWithInverse(permutation);

  HessianFactor::shared_ptr sorted_factor2(
      new HessianFactor(JacobianFactor(0,
                        Matrix_(6,3,
                                25.7429,   -1.6897,    0.4587,
                                 1.6400,   23.3095,   -8.4816,
                                 0.0034,    0.0509,  -25.7855,
                                 0.9997,   -0.0002,    0.0824,
                                    0.0,    0.9973,    0.9517,
                                    0.0,       0.0,    0.9973),
                        1,
                        Matrix_(6,3,
                                25.8367,    0.1211,    0.0593,
                                    0.0,   23.4099,   30.8733,
                                    0.0,       0.0,   25.8729,
                                    0.0,       0.0,       0.0,
                                    0.0,       0.0,       0.0,
                                    0.0,       0.0,       0.0),
                        Vector_(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        noiseModel::Unit::Create(6))));

  GaussianFactorGraph sortedGraph;
//  sortedGraph.push_back(factor1);
  sortedGraph.push_back(sorted_factor2);

  GaussianFactorGraph unsortedGraph;
//  unsortedGraph.push_back(factor1);
  unsortedGraph.push_back(unsorted_factor2);

  GaussianBayesNet::shared_ptr expected_bn;
  GaussianFactor::shared_ptr expected_factor;
  boost::tie(expected_bn, expected_factor) =
  		EliminatePreferCholesky(sortedGraph, 1);

  GaussianBayesNet::shared_ptr actual_bn;
  GaussianFactor::shared_ptr actual_factor;
  boost::tie(actual_bn, actual_factor) =
  		EliminatePreferCholesky(unsortedGraph, 1);

  CHECK(assert_equal(*expected_bn, *actual_bn, 1e-10));
  CHECK(assert_equal(*expected_factor, *actual_factor, 1e-10));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
