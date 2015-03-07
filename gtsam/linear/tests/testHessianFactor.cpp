/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testCholeskyFactor.cpp
 * @author  Richard Roberts
 * @date    Dec 15, 2010
 */

#include <vector>
#include <utility>

#include <boost/assign/std/vector.hpp>
#include <boost/assign/std/map.hpp>

#include <gtsam/base/debug.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/GaussianFactorGraph.h>

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace boost::assign;
using namespace gtsam;

const double tol = 1e-5;

/* ************************************************************************* */
TEST(HessianFactor, emptyConstructor) {
	HessianFactor f;
	DOUBLES_EQUAL(0.0, f.constantTerm(), 1e-9);        // Constant term should be zero
	EXPECT(assert_equal(Vector(), f.linearTerm()));    // Linear term should be empty
	EXPECT(assert_equal(zeros(1,1), f.info()));        // Full matrix should be 1-by-1 zero matrix
	DOUBLES_EQUAL(0.0, f.error(VectorValues()), 1e-9); // Should have zero error
}

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
  JacobianFactor combined(meas, b2, noiseModel::Diagonal::Sigmas(sigmas));

  HessianFactor actual(combined);

  VectorValues values(std::vector<size_t>(dims, dims+2));
  values[0] = Vector_(2, 1.0, 2.0);
  values[1] = Vector_(4, 3.0, 4.0, 5.0, 6.0);

  EXPECT_LONGS_EQUAL(2, actual.size());

  EXPECT(assert_equal(expected, actual, 1e-9));

  // error terms
  EXPECT_DOUBLES_EQUAL(combined.error(values), actual.error(values), 1e-9);
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

  // extract underlying parts
  Matrix info_matrix = factor.info_.range(0, 1, 0, 1);
  EXPECT(assert_equal(Matrix(G), info_matrix));
  EXPECT_DOUBLES_EQUAL(f, factor.constantTerm(), 1e-10);
  EXPECT(assert_equal(g, Vector(factor.linearTerm()), 1e-10));
  EXPECT_LONGS_EQUAL(1, factor.size());

  // error 0.5*(f - 2*x'*g + x'*G*x)
  double expected = 80.375;
  double actual = factor.error(dx);
  double expected_manual = 0.5 * (f - 2.0 * dxv.dot(g) + dxv.transpose() * G.selfadjointView<Eigen::Upper>() * dxv);
  EXPECT_DOUBLES_EQUAL(expected, expected_manual, 1e-10);
  DOUBLES_EQUAL(expected, actual, 1e-10);
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor1b)
{
	Vector mu = Vector_(2,1.0,2.0);
	Matrix Sigma = eye(2,2);

  HessianFactor factor(0, mu, Sigma);

  Matrix G = eye(2,2);
  Vector g = G*mu;
  double f = dot(g,mu);

  // Check
  Matrix info_matrix = factor.info_.range(0, 1, 0, 1);
  EXPECT(assert_equal(Matrix(G), info_matrix));
  EXPECT_DOUBLES_EQUAL(f, factor.constantTerm(), 1e-10);
  EXPECT(assert_equal(g, Vector(factor.linearTerm()), 1e-10));
  EXPECT_LONGS_EQUAL(1, factor.size());
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
  LONGS_EQUAL(4, factor.rows());
  DOUBLES_EQUAL(10.0, factor.constantTerm(), 1e-10);

  Vector linearExpected(3);  linearExpected << g1, g2;
  EXPECT(assert_equal(linearExpected, factor.linearTerm()));

  EXPECT(assert_equal(G11, factor.info(factor.begin(), factor.begin())));
  EXPECT(assert_equal(G12, factor.info(factor.begin(), factor.begin()+1)));
  EXPECT(assert_equal(G22, factor.info(factor.begin()+1, factor.begin()+1)));
}

/* ************************************************************************* */
TEST(HessianFactor, Constructor3)
{
  Matrix G11 = Matrix_(1,1, 1.0);
  Matrix G12 = Matrix_(1,2, 2.0, 4.0);
  Matrix G13 = Matrix_(1,3, 3.0, 6.0, 9.0);

  Matrix G22 = Matrix_(2,2, 3.0, 5.0, 0.0, 6.0);
  Matrix G23 = Matrix_(2,3, 4.0, 6.0, 8.0, 1.0, 2.0, 4.0);

  Matrix G33 = Matrix_(3,3, 1.0, 2.0, 3.0, 0.0, 5.0, 6.0, 0.0, 0.0, 9.0);

  Vector g1 = Vector_(1, -7.0);
  Vector g2 = Vector_(2, -8.0, -9.0);
  Vector g3 = Vector_(3,  1.0,  2.0,  3.0);

  double f = 10.0;

  Vector dx0 = Vector_(1, 0.5);
  Vector dx1 = Vector_(2, 1.5, 2.5);
  Vector dx2 = Vector_(3, 1.5, 2.5, 3.5);

  vector<size_t> dims;
  dims.push_back(1);
  dims.push_back(2);
  dims.push_back(3);
  VectorValues dx(dims);

  dx[0] = dx0;
  dx[1] = dx1;
  dx[2] = dx2;

  HessianFactor factor(0, 1, 2, G11, G12, G13, g1, G22, G23, g2, G33, g3, f);

  double expected = 371.3750;
  double actual = factor.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
  LONGS_EQUAL(7, factor.rows());
  DOUBLES_EQUAL(10.0, factor.constantTerm(), 1e-10);

  Vector linearExpected(6);  linearExpected << g1, g2, g3;
  EXPECT(assert_equal(linearExpected, factor.linearTerm()));

  EXPECT(assert_equal(G11, factor.info(factor.begin()+0, factor.begin()+0)));
  EXPECT(assert_equal(G12, factor.info(factor.begin()+0, factor.begin()+1)));
  EXPECT(assert_equal(G13, factor.info(factor.begin()+0, factor.begin()+2)));
  EXPECT(assert_equal(G22, factor.info(factor.begin()+1, factor.begin()+1)));
  EXPECT(assert_equal(G23, factor.info(factor.begin()+1, factor.begin()+2)));
  EXPECT(assert_equal(G33, factor.info(factor.begin()+2, factor.begin()+2)));
}

/* ************************************************************************* */
TEST(HessianFactor, ConstructorNWay)
{
  Matrix G11 = Matrix_(1,1, 1.0);
  Matrix G12 = Matrix_(1,2, 2.0, 4.0);
  Matrix G13 = Matrix_(1,3, 3.0, 6.0, 9.0);

  Matrix G22 = Matrix_(2,2, 3.0, 5.0, 0.0, 6.0);
  Matrix G23 = Matrix_(2,3, 4.0, 6.0, 8.0, 1.0, 2.0, 4.0);

  Matrix G33 = Matrix_(3,3, 1.0, 2.0, 3.0, 0.0, 5.0, 6.0, 0.0, 0.0, 9.0);

  Vector g1 = Vector_(1, -7.0);
  Vector g2 = Vector_(2, -8.0, -9.0);
  Vector g3 = Vector_(3,  1.0,  2.0,  3.0);

  double f = 10.0;

  Vector dx0 = Vector_(1, 0.5);
  Vector dx1 = Vector_(2, 1.5, 2.5);
  Vector dx2 = Vector_(3, 1.5, 2.5, 3.5);

  vector<size_t> dims;
  dims.push_back(1);
  dims.push_back(2);
  dims.push_back(3);
  VectorValues dx(dims);

  dx[0] = dx0;
  dx[1] = dx1;
  dx[2] = dx2;


  std::vector<Index> js;
  js.push_back(0); js.push_back(1); js.push_back(2);
  std::vector<Matrix> Gs;
  Gs.push_back(G11); Gs.push_back(G12); Gs.push_back(G13); Gs.push_back(G22); Gs.push_back(G23); Gs.push_back(G33);
  std::vector<Vector> gs;
  gs.push_back(g1); gs.push_back(g2); gs.push_back(g3);
  HessianFactor factor(js, Gs, gs, f);

  double expected = 371.3750;
  double actual = factor.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
  LONGS_EQUAL(7, factor.rows());
  DOUBLES_EQUAL(10.0, factor.constantTerm(), 1e-10);

  Vector linearExpected(6);  linearExpected << g1, g2, g3;
  EXPECT(assert_equal(linearExpected, factor.linearTerm()));

  EXPECT(assert_equal(G11, factor.info(factor.begin()+0, factor.begin()+0)));
  EXPECT(assert_equal(G12, factor.info(factor.begin()+0, factor.begin()+1)));
  EXPECT(assert_equal(G13, factor.info(factor.begin()+0, factor.begin()+2)));
  EXPECT(assert_equal(G22, factor.info(factor.begin()+1, factor.begin()+1)));
  EXPECT(assert_equal(G23, factor.info(factor.begin()+1, factor.begin()+2)));
  EXPECT(assert_equal(G33, factor.info(factor.begin()+2, factor.begin()+2)));
}

/* ************************************************************************* */
TEST_UNSAFE(HessianFactor, CopyConstructor_and_assignment)
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

  HessianFactor originalFactor(0, 1, G11, G12, g1, G22, g2, f);

  // Make a copy
  HessianFactor copy1(originalFactor);

  double expected = 90.5;
  double actual = copy1.error(dx);

  DOUBLES_EQUAL(expected, actual, 1e-10);
  LONGS_EQUAL(4, copy1.rows());
  DOUBLES_EQUAL(10.0, copy1.constantTerm(), 1e-10);

  Vector linearExpected(3);  linearExpected << g1, g2;
  EXPECT(assert_equal(linearExpected, copy1.linearTerm()));

  EXPECT(assert_equal(G11, copy1.info(copy1.begin(), copy1.begin())));
  EXPECT(assert_equal(G12, copy1.info(copy1.begin(), copy1.begin()+1)));
  EXPECT(assert_equal(G22, copy1.info(copy1.begin()+1, copy1.begin()+1)));

  // Make a copy using the assignment operator
  HessianFactor copy2;
  copy2 = HessianFactor(originalFactor); // Make a temporary to make sure copying does not shared references

  actual = copy2.error(dx);
  DOUBLES_EQUAL(expected, actual, 1e-10);
  LONGS_EQUAL(4, copy2.rows());
  DOUBLES_EQUAL(10.0, copy2.constantTerm(), 1e-10);

  EXPECT(assert_equal(linearExpected, copy2.linearTerm()));

  EXPECT(assert_equal(G11, copy2.info(copy2.begin(), copy2.begin())));
  EXPECT(assert_equal(G12, copy2.info(copy2.begin(), copy2.begin()+1)));
  EXPECT(assert_equal(G22, copy2.info(copy2.begin()+1, copy2.begin()+1)));
}

/* ************************************************************************* */
TEST_UNSAFE(HessianFactor, CombineAndEliminate)
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

  // create a full, uneliminated version of the factor
  JacobianFactor expectedFactor(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));

  // perform elimination
  GaussianConditional::shared_ptr expectedBN = expectedFactor.eliminate(1);

  // create expected Hessian after elimination
  HessianFactor expectedCholeskyFactor(expectedFactor);

  // Convert all factors to hessians
  FactorGraph<HessianFactor> hessians;
  BOOST_FOREACH(const GaussianFactorGraph::sharedFactor& factor, gfg) {
    if(boost::shared_ptr<HessianFactor> hf = boost::dynamic_pointer_cast<HessianFactor>(factor))
      hessians.push_back(hf);
    else if(boost::shared_ptr<JacobianFactor> jf = boost::dynamic_pointer_cast<JacobianFactor>(factor))
      hessians.push_back(boost::make_shared<HessianFactor>(*jf));
    else
      CHECK(false);
  }

  // Eliminate
  GaussianFactorGraph::EliminationResult actualCholesky = EliminateCholesky(gfg, 1);
	HessianFactor::shared_ptr actualFactor = boost::dynamic_pointer_cast<HessianFactor>(actualCholesky.second);

	EXPECT(assert_equal(*expectedBN, *actualCholesky.first, 1e-6));
  EXPECT(assert_equal(expectedCholeskyFactor, *actualFactor, 1e-6));
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
  JacobianFactor combined(meas, b2, noiseModel::Diagonal::Sigmas(sigmas));

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
  EXPECT(assert_equal(expectedCG,*actual_Chol.first,1e-4));

  // the expected linear factor
  double sigma = 0.2236;
  Matrix Bl1x1 = Matrix_(2,4,
      // l1          x1
      1.00, 0.00, -1.00,  0.00,
      0.00, 1.00, +0.00, -1.00
  )/sigma;
  Vector b1 = Vector_(2,0.0,0.894427);
  JacobianFactor expectedLF(1, Bl1x1, b1, noiseModel::Isotropic::Sigma(2,1.0));
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

  GaussianConditional::shared_ptr expected_bn;
  GaussianFactor::shared_ptr expected_factor;
  boost::tie(expected_bn, expected_factor) =
  		EliminatePreferCholesky(sortedGraph, 1);

  GaussianConditional::shared_ptr actual_bn;
  GaussianFactor::shared_ptr actual_factor;
  boost::tie(actual_bn, actual_factor) =
  		EliminatePreferCholesky(unsortedGraph, 1);

  EXPECT(assert_equal(*expected_bn, *actual_bn, 1e-10));
  EXPECT(assert_equal(*expected_factor, *actual_factor, 1e-10));
}

/* ************************************************************************* */
TEST(HessianFactor, combine) {

	// update the information matrix with a single jacobian factor
	Matrix A0 = Matrix_(2, 2,
	11.1803399,     0.0,
	    0.0, 11.1803399);
	Matrix A1 = Matrix_(2, 2,
	-2.23606798,        0.0,
	       0.0, -2.23606798);
	Matrix A2 = Matrix_(2, 2,
	-8.94427191,      0.0,
				 0.0, -8.94427191);
	Vector b = Vector_(2, 2.23606798,-1.56524758);
	SharedDiagonal model = noiseModel::Diagonal::Sigmas(ones(2));
	GaussianFactor::shared_ptr f(new JacobianFactor(0, A0, 1, A1, 2, A2, b, model));
	FactorGraph<GaussianFactor> factors;
	factors.push_back(f);

	vector<size_t> dimensions;
	dimensions += 2, 2, 2, 1;

	Scatter scatter;
	scatter += make_pair(0, SlotEntry(0, 2));
	scatter += make_pair(1, SlotEntry(1, 2));
	scatter += make_pair(2, SlotEntry(2, 2));

	// Form Ab' * Ab
	HessianFactor actual(factors, dimensions, scatter);

	Matrix expected = Matrix_(7, 7,
  125.0000,       0.0,  -25.0000,       0.0, -100.0000,       0.0,   25.0000,
       0.0,  125.0000,       0.0,  -25.0000,       0.0, -100.0000,  -17.5000,
  -25.0000,       0.0,    5.0000,       0.0,   20.0000,       0.0,   -5.0000,
       0.0,  -25.0000,       0.0,    5.0000,       0.0,   20.0000,    3.5000,
 -100.0000,       0.0,   20.0000,       0.0,   80.0000,       0.0,  -20.0000,
       0.0, -100.0000,       0.0,   20.0000,       0.0,   80.0000,   14.0000,
   25.0000,  -17.5000,   -5.0000,    3.5000,  -20.0000,   14.0000,    7.4500);
	EXPECT(assert_equal(Matrix(expected.triangularView<Eigen::Upper>()),
			Matrix(actual.matrix_.triangularView<Eigen::Upper>()), tol));

}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
