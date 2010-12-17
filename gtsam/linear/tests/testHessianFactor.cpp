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

  EXPECT(assert_equal(expected, actual, 1e-9));

}

/* ************************************************************************* */
TEST_UNSAFE(GaussianFactor, CombineAndEliminate)
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

  pair<GaussianBayesNet::shared_ptr, HessianFactor::shared_ptr> actualCholesky(HessianFactor::CombineAndEliminate(
      *gfg.convertCastFactors<FactorGraph<HessianFactor> >(), 1));

  EXPECT(assert_equal(expectedBN, *actualCholesky.first, 1e-6));
  EXPECT(assert_equal(HessianFactor(expectedFactor), *actualCholesky.second, 1e-6));
}

/* ************************************************************************* */
TEST(GaussianFactor, eliminate2 )
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
  pair<GaussianBayesNet::shared_ptr, HessianFactor::shared_ptr> actual_Chol =
      HessianFactor::CombineAndEliminate(combinedLFG_Chol, 1);

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
  EXPECT(assert_equal(HessianFactor(expectedLF), *actual_Chol.second, 1.5e-3));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
