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
#include <gtsam/inference/VariableIndex.h>
#include <gtsam/linear/GaussianFactorGraph.h>

using namespace std;
using namespace gtsam;
using namespace boost;

#ifdef BROKEN
static const Index _x0_=0, _x1_=1, _x2_=2, _x3_=3, _x4_=4, _x_=5, _y_=6, _l1_=7, _l11_=8;
#endif

static SharedDiagonal
	sigma0_1 = noiseModel::Isotropic::Sigma(2,0.1), sigma_02 = noiseModel::Isotropic::Sigma(2,0.2),
	constraintModel = noiseModel::Constrained::All(2);

/* ************************************************************************* */
TEST(GaussianFactorGraph, initialization) {
	// Create empty graph
	GaussianFactorGraph fg;
	SharedDiagonal unit2 = noiseModel::Unit::Create(2);

	fg.add(0, 10*eye(2), -1.0*ones(2), unit2);
	fg.add(0, -10*eye(2),1, 10*eye(2), Vector_(2, 2.0, -1.0), unit2);
	fg.add(0, -5*eye(2), 2, 5*eye(2), Vector_(2, 0.0, 1.0), unit2);
	fg.add(1, -5*eye(2), 2, 5*eye(2), Vector_(2, -1.0, 1.5), unit2);

	EXPECT_LONGS_EQUAL(4, fg.size());
	JacobianFactor factor = *boost::dynamic_pointer_cast<JacobianFactor>(fg[0]);

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
#ifdef BROKEN
TEST(GaussianFactor, Combine)
{
  Matrix A00 = Matrix_(3,3,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0);
  Vector b0 = Vector_(3, 0.0, 0.0, 0.0);
  Vector s0 = Vector_(3, 0.0, 0.0, 0.0);

  Matrix A10 = Matrix_(3,3,
      0.0, -2.0, -4.0,
      2.0, 0.0,  2.0,
      0.0, 0.0, -10.0);
  Matrix A11 = Matrix_(3,3,
      2.0, 0.0, 0.0,
      0.0, 2.0, 0.0,
      0.0, 0.0, 10.0);
  Vector b1 = Vector_(3, 6.0, 2.0, 0.0);
  Vector s1 = Vector_(3, 1.0, 1.0, 1.0);

  Matrix A20 = Matrix_(3,3,
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0);
  Vector b2 = Vector_(3, 0.0, 0.0, 0.0);
  Vector s2 = Vector_(3, 100.0, 100.0, 100.0);

  GaussianFactorGraph gfg;
  gfg.add(0, A00, b0, noiseModel::Diagonal::Sigmas(s0, true));
  gfg.add(0, A10, 1, A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
  gfg.add(0, A20, b2, noiseModel::Diagonal::Sigmas(s2, true));

  GaussianVariableIndex varindex(gfg);
  vector<size_t> factors(3); factors[0]=0; factors[1]=1; factors[2]=2;
  vector<size_t> variables(2); variables[0]=0; variables[1]=1;
  vector<vector<size_t> > variablePositions(3);
  variablePositions[0].resize(1); variablePositions[0][0]=0;
  variablePositions[1].resize(2); variablePositions[1][0]=0; variablePositions[1][1]=1;
  variablePositions[2].resize(1); variablePositions[2][0]=0;

  JacobianFactor actual = *JacobianFactor::Combine(gfg, varindex, factors, variables, variablePositions);

  Matrix zero3x3 = zeros(3,3);
  Matrix A0 = gtsam::stack(3, &A00, &A10, &A20);
  Matrix A1 = gtsam::stack(3, &zero3x3, &A11, &zero3x3);
  Vector b = gtsam::concatVectors(3, &b0, &b1, &b2);
  Vector sigmas = gtsam::concatVectors(3, &s0, &s1, &s2);

  JacobianFactor expected(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));

  EXPECT(assert_equal(expected, actual));
}
#endif

/* ************************************************************************* */
TEST(GaussianFactorGraph, Combine2)
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

  // Convert to Jacobians, inefficient copy of all factors instead of selectively converting only Hessians
  FactorGraph<JacobianFactor> jacobians;
  BOOST_FOREACH(const GaussianFactorGraph::sharedFactor& factor, gfg) {
    jacobians.push_back(boost::make_shared<JacobianFactor>(*factor));
  }

  // Combine Jacobians into a single dense factor
  JacobianFactor actual = *CombineJacobians(jacobians, VariableSlots(gfg));

  Matrix zero3x3 = zeros(3,3);
  Matrix A0 = gtsam::stack(3, &zero3x3, &A10, &zero3x3);
  Matrix A1 = gtsam::stack(3, &A01, &A11, &A21);
  Vector b = gtsam::concatVectors(3, &b0, &b1, &b2);
  Vector sigmas = gtsam::concatVectors(3, &s0, &s1, &s2);

  JacobianFactor expected(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));

  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactor, CombineAndEliminate)
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
  GaussianConditional::shared_ptr expectedBN = expectedFactor.eliminate(1);

  GaussianConditional::shared_ptr actualBN;
	GaussianFactor::shared_ptr actualFactor;
	boost::tie(actualBN, actualFactor) = EliminateQR(gfg, 1);
	JacobianFactor::shared_ptr actualJacobian = boost::dynamic_pointer_cast<
			JacobianFactor>(actualFactor);

  EXPECT(assert_equal(*expectedBN, *actualBN));
  EXPECT(assert_equal(expectedFactor, *actualJacobian));
}

/* ************************************************************************* */
#ifdef BROKEN
TEST( NonlinearFactorGraph, combine2){
	double sigma1 = 0.0957;
	Matrix A11(2,2);
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0;       A11(1,1) = 1;
	Vector b(2);
	b(0) = 2; b(1) = -1;
	JacobianFactor::shared_ptr f1(new JacobianFactor(_x1_, A11, b*sigma1, noiseModel::Isotropic::Sigma(2,sigma1)));

	double sigma2 = 0.5;
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -1;
	b(0) = 4 ; b(1) = -5;
	JacobianFactor::shared_ptr f2(new JacobianFactor(_x1_, A11, b*sigma2, noiseModel::Isotropic::Sigma(2,sigma2)));

	double sigma3 = 0.25;
	A11(0,0) = 1; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = -1;
	b(0) = 3 ; b(1) = -88;
	JacobianFactor::shared_ptr f3(new JacobianFactor(_x1_, A11, b*sigma3, noiseModel::Isotropic::Sigma(2,sigma3)));

	// TODO: find a real sigma value for this example
	double sigma4 = 0.1;
	A11(0,0) = 6; A11(0,1) =  0;
	A11(1,0) = 0; A11(1,1) = 7;
	b(0) = 5 ; b(1) = -6;
	JacobianFactor::shared_ptr f4(new JacobianFactor(_x1_, A11*sigma4, b*sigma4, noiseModel::Isotropic::Sigma(2,sigma4)));

	vector<JacobianFactor::shared_ptr> lfg;
	lfg.push_back(f1);
	lfg.push_back(f2);
	lfg.push_back(f3);
	lfg.push_back(f4);
	JacobianFactor combined(lfg);

	Vector sigmas = Vector_(8, sigma1, sigma1, sigma2, sigma2, sigma3, sigma3, sigma4, sigma4);
	Matrix A22(8,2);
	A22(0,0) = 1;   A22(0,1) =  0;
	A22(1,0) = 0;   A22(1,1) = 1;
	A22(2,0) = 1;   A22(2,1) =  0;
	A22(3,0) = 0;   A22(3,1) = -1;
	A22(4,0) = 1;   A22(4,1) =  0;
	A22(5,0) = 0;   A22(5,1) = -1;
	A22(6,0) = 0.6; A22(6,1) =  0;
	A22(7,0) = 0;   A22(7,1) =  0.7;
	Vector exb(8);
	exb(0) = 2*sigma1 ; exb(1) = -1*sigma1;  exb(2) = 4*sigma2 ; exb(3) = -5*sigma2;
	exb(4) = 3*sigma3 ; exb(5) = -88*sigma3; exb(6) = 5*sigma4 ; exb(7) = -6*sigma4;

	vector<pair<Index, Matrix> > meas;
	meas.push_back(make_pair(_x1_, A22));
	JacobianFactor expected(meas, exb, sigmas);
	EXPECT(assert_equal(expected,combined));
}

/* ************************************************************************* */
TEST(GaussianFactor, linearFactorN){
	Matrix I = eye(2);
  vector<JacobianFactor::shared_ptr> f;
  SharedDiagonal model = noiseModel::Isotropic::Sigma(2,1.0);
  f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x1_, I, Vector_(2,
			10.0, 5.0), model)));
	f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x1_, -10 * I,
			_x2_, 10 * I, Vector_(2, 1.0, -2.0), model)));
	f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x2_, -10 * I,
			_x3_, 10 * I, Vector_(2, 1.5, -1.5), model)));
	f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x3_, -10 * I,
			_x4_, 10 * I, Vector_(2, 2.0, -1.0), model)));

  JacobianFactor combinedFactor(f);

  vector<pair<Index, Matrix> > combinedMeasurement;
  combinedMeasurement.push_back(make_pair(_x1_, Matrix_(8,2,
      1.0,  0.0,
      0.0,  1.0,
    -10.0,  0.0,
      0.0,-10.0,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0)));
  combinedMeasurement.push_back(make_pair(_x2_, Matrix_(8,2,
      0.0,  0.0,
      0.0,  0.0,
     10.0,  0.0,
      0.0, 10.0,
    -10.0,  0.0,
      0.0,-10.0,
      0.0,  0.0,
      0.0,  0.0)));
  combinedMeasurement.push_back(make_pair(_x3_, Matrix_(8,2,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0,
      0.0,  0.0,
     10.0,  0.0,
      0.0, 10.0,
    -10.0,  0.0,
      0.0,-10.0)));
  combinedMeasurement.push_back(make_pair(_x4_, Matrix_(8,2,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
      0.0, 0.0,
     10.0, 0.0,
      0.0,10.0)));
  Vector b = Vector_(8,
      10.0, 5.0, 1.0, -2.0, 1.5, -1.5, 2.0, -1.0);

  Vector sigmas = repeat(8,1.0);
  JacobianFactor expected(combinedMeasurement, b, sigmas);
  EXPECT(assert_equal(expected,combinedFactor));
}
#endif

/* ************************************************************************* */
TEST(GaussianFactor, eliminateFrontals)
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
  JacobianFactor::shared_ptr factor1(new JacobianFactor(terms1, b1, noiseModel::Isotropic::Sigma(4, 0.5)));

  // Create second factor
  list<pair<Index, Matrix> > terms2;
  terms2 +=
      make_pair(5, Matrix(Ab.block(4, 2, 4, 2))),
      make_pair(7, Matrix(Ab.block(4, 4, 4, 2))),
      make_pair(9, Matrix(Ab.block(4, 6, 4, 2))),
      make_pair(11,Matrix(Ab.block(4, 8, 4, 2)));
  Vector b2 = Ab.col(10).segment(4, 4);
  JacobianFactor::shared_ptr factor2(new JacobianFactor(terms2, b2, noiseModel::Isotropic::Sigma(4, 0.5)));

  // Create third factor
  list<pair<Index, Matrix> > terms3;
  terms3 +=
      make_pair(7, Matrix(Ab.block(8, 4, 4, 2))),
      make_pair(9, Matrix(Ab.block(8, 6, 4, 2))),
      make_pair(11,Matrix(Ab.block(8, 8, 4, 2)));
  Vector b3 = Ab.col(10).segment(8, 4);
  JacobianFactor::shared_ptr factor3(new JacobianFactor(terms3, b3, noiseModel::Isotropic::Sigma(4, 0.5)));

  // Create fourth factor
  list<pair<Index, Matrix> > terms4;
  terms4 +=
  		make_pair(11, Matrix(Ab.block(12, 8, 2, 2)));
  Vector b4 = Ab.col(10).segment(12, 2);
  JacobianFactor::shared_ptr factor4(new JacobianFactor(terms4, b4, noiseModel::Isotropic::Sigma(2, 0.5)));

  // Create factor graph
  GaussianFactorGraph factors;
  factors.push_back(factor1);
  factors.push_back(factor2);
  factors.push_back(factor3);
  factors.push_back(factor4);

  // extract the dense matrix for the graph
  Matrix actualDense = factors.augmentedJacobian();
  EXPECT(assert_equal(2.0 * Ab, actualDense));

  // Convert to Jacobians, inefficient copy of all factors instead of selectively converting only Hessians
  FactorGraph<JacobianFactor> jacobians;
  BOOST_FOREACH(const GaussianFactorGraph::sharedFactor& factor, factors) {
    jacobians.push_back(boost::make_shared<JacobianFactor>(*factor));
  }

  // Create combined factor
  JacobianFactor combined(*CombineJacobians(jacobians, VariableSlots(factors)));

  // Copies factors as they will be eliminated in place
  JacobianFactor actualFactor_QR = combined;
  JacobianFactor actualFactor_Chol = combined;

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
  GaussianConditional::shared_ptr cond1(new GaussianConditional(3, d1, R1, cterms1, ones(2)));

  // Expected conditional on second variable from next 2 rows of R
  Matrix R2 = sub(R, 2,4, 2,4);
  list<pair<Index, Matrix> > cterms2;
  cterms2 +=
      make_pair(7, sub(R, 2,4, 4,6)),
      make_pair(9, sub(R, 2,4, 6,8)),
      make_pair(11,sub(R, 2,4, 8,10));
  Vector d2 = R.col(10).segment(2,2);
  GaussianConditional::shared_ptr cond2(new GaussianConditional(5, d2, R2, cterms2, ones(2)));

  // Expected conditional on third variable from next 2 rows of R
  Matrix R3 = sub(R, 4,6, 4,6);
  list<pair<Index, Matrix> > cterms3;
  cterms3 +=
      make_pair(9,  sub(R, 4,6, 6,8)),
      make_pair(11, sub(R, 4,6, 8,10));
  Vector d3 = R.col(10).segment(4,2);
  GaussianConditional::shared_ptr cond3(new GaussianConditional(7, d3, R3, cterms3, ones(2)));

  // Create expected Bayes net fragment from three conditionals above
//  GaussianBayesNet expectedFragment;
//  expectedFragment.push_back(cond1);
//  expectedFragment.push_back(cond2);
//  expectedFragment.push_back(cond3);
  Index ikeys[] = {3,5,7,9,11};
  std::vector<Index> keys(ikeys, ikeys + sizeof(ikeys)/sizeof(Index));
  size_t dims[] = { 2,2,2,2,2,1 };
  size_t height = 11;
  VerticalBlockView<Matrix> Rblock(R, dims, dims+6, height);
  GaussianConditional::shared_ptr expectedFragment( new GaussianConditional(keys.begin(), keys.end(), 3,
      Rblock, ones(6)) );

  // Get expected matrices for remaining factor
  Matrix Ae1 = sub(R, 6,10, 6,8);
  Matrix Ae2 = sub(R, 6,10, 8,10);
  Vector be = R.col(10).segment(6, 4);

  // Eliminate (3 frontal variables, 6 scalar columns) using QR !!!!
  GaussianConditional::shared_ptr actualFragment_QR = actualFactor_QR.eliminate(3);

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
  GaussianBayesNet actualFragment_Chol = *actualFactor_Chol.eliminate(3, JacobianFactor::SOLVE_CHOLESKY);
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
#ifdef BROKEN
TEST(GaussianFactor, CONSTRUCTOR_GaussianConditionalConstrained )
{
  Matrix Ax = eye(2);
  Vector b = Vector_(2, 3.0, 5.0);
  SharedDiagonal noisemodel = noiseModel::Constrained::All(2);
  JacobianFactor::shared_ptr expected(new JacobianFactor(_x0_, Ax, b, noisemodel));
  GaussianFactorGraph graph;
  graph.push_back(expected);

  GaussianConditional::shared_ptr conditional = GaussianSequentialSolver::EliminateUntil(graph,_x0_+1);
  JacobianFactor actual(*conditional);

  EXPECT(assert_equal(*expected, actual));
}
#endif

/* ************************************************************************* */
TEST(GaussianFactor, permuteWithInverse)
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

  JacobianFactor actual(1, A1, 3, A2, 5, A3, b, noiseModel::Isotropic::Sigma(2, 1.0));
  GaussianFactorGraph actualFG; actualFG.push_back(JacobianFactor::shared_ptr(new JacobianFactor(actual)));
  VariableIndex actualIndex(actualFG);
  actual.permuteWithInverse(inversePermutation);
//  actualIndex.permute(*inversePermutation.inverse());

  JacobianFactor expected(4, A1, 2, A2, 0, A3, b, noiseModel::Isotropic::Sigma(2, 1.0));
  GaussianFactorGraph expectedFG; expectedFG.push_back(JacobianFactor::shared_ptr(new JacobianFactor(expected)));
//  GaussianVariableIndex expectedIndex(expectedFG);

  EXPECT(assert_equal(expected, actual));

#ifdef BROKEN
  // todo: fix this!!!  VariableIndex should not hold slots
  for(Index j=0; j<actualIndex.size(); ++j) {
    BOOST_FOREACH( GaussianVariableIndex::mapped_factor_type& factor_pos, actualIndex[j]) {
      factor_pos.variablePosition = numeric_limits<Index>::max(); }
  }
  for(Index j=0; j<expectedIndex.size(); ++j) {
    BOOST_FOREACH( GaussianVariableIndex::mapped_factor_type& factor_pos, expectedIndex[j]) {
      factor_pos.variablePosition = numeric_limits<Index>::max(); }
  }
  EXPECT(assert_equal(expectedIndex, actualIndex));
#endif
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
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
