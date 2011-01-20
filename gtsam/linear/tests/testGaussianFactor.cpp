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

#include <iostream>
#include <vector>

#include <boost/tuple/tuple.hpp>
#include <boost/assign/std/list.hpp> // for operator +=
#include <boost/assign/std/set.hpp>
#include <boost/assign/std/map.hpp> // for insert
using namespace boost::assign;

#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

//#define GTSAM_MAGIC_KEY

#include <gtsam/base/types.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/linear/GaussianSequentialSolver.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/inference/VariableSlots.h>

using namespace std;
using namespace gtsam;
using namespace boost;
namespace ublas = boost::numeric::ublas;

static const Index _x0_=0, _x1_=1, _x2_=2, _x3_=3, _x4_=4, _x_=5, _y_=6, _l1_=7, _l11_=8;

static SharedDiagonal
	sigma0_1 = sharedSigma(2,0.1), sigma_02 = sharedSigma(2,0.2),
	constraintModel = noiseModel::Constrained::All(2);

/* ************************************************************************* */
TEST(GaussianFactor, constructor)
{
	Vector b = Vector_(3, 1., 2., 3.);
	SharedDiagonal noise = noiseModel::Diagonal::Sigmas(Vector_(3,1.,1.,1.));
	std::list<std::pair<Index, Matrix> > terms;
	terms.push_back(make_pair(_x0_, eye(3)));
	terms.push_back(make_pair(_x1_, 2.*eye(3)));
	JacobianFactor actual(terms, b, noise);
	JacobianFactor expected(_x0_, eye(3), _x1_, 2.*eye(3), b, noise);
	EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(GaussianFactor, constructor2)
{
  Vector b = Vector_(3, 1., 2., 3.);
  SharedDiagonal noise = noiseModel::Diagonal::Sigmas(Vector_(3,1.,1.,1.));
  std::list<std::pair<Index, Matrix> > terms;
  terms.push_back(make_pair(_x0_, eye(3)));
  terms.push_back(make_pair(_x1_, 2.*eye(3)));
  const JacobianFactor actual(terms, b, noise);

  JacobianFactor::const_iterator key0 = actual.begin();
  JacobianFactor::const_iterator key1 = key0 + 1;
  EXPECT(assert_equal(*key0, _x0_));
  EXPECT(assert_equal(*key1, _x1_));

  Matrix actualA0 = actual.getA(key0);
  Matrix actualA1 = actual.getA(key1);
  Vector actualb = actual.getb();

  EXPECT(assert_equal(eye(3), actualA0));
  EXPECT(assert_equal(2.*eye(3), actualA1));
  EXPECT(assert_equal(b, actualb));
}

///* ************************************************************************* */
//TEST(GaussianFactor, Combine)
//{
//  Matrix A00 = Matrix_(3,3,
//      1.0, 0.0, 0.0,
//      0.0, 1.0, 0.0,
//      0.0, 0.0, 1.0);
//  Vector b0 = Vector_(3, 0.0, 0.0, 0.0);
//  Vector s0 = Vector_(3, 0.0, 0.0, 0.0);
//
//  Matrix A10 = Matrix_(3,3,
//      0.0, -2.0, -4.0,
//      2.0, 0.0,  2.0,
//      0.0, 0.0, -10.0);
//  Matrix A11 = Matrix_(3,3,
//      2.0, 0.0, 0.0,
//      0.0, 2.0, 0.0,
//      0.0, 0.0, 10.0);
//  Vector b1 = Vector_(3, 6.0, 2.0, 0.0);
//  Vector s1 = Vector_(3, 1.0, 1.0, 1.0);
//
//  Matrix A20 = Matrix_(3,3,
//      1.0, 0.0, 0.0,
//      0.0, 1.0, 0.0,
//      0.0, 0.0, 1.0);
//  Vector b2 = Vector_(3, 0.0, 0.0, 0.0);
//  Vector s2 = Vector_(3, 100.0, 100.0, 100.0);
//
//  GaussianFactorGraph gfg;
//  gfg.add(0, A00, b0, noiseModel::Diagonal::Sigmas(s0, true));
//  gfg.add(0, A10, 1, A11, b1, noiseModel::Diagonal::Sigmas(s1, true));
//  gfg.add(0, A20, b2, noiseModel::Diagonal::Sigmas(s2, true));
//
//  GaussianVariableIndex varindex(gfg);
//  vector<size_t> factors(3); factors[0]=0; factors[1]=1; factors[2]=2;
//  vector<size_t> variables(2); variables[0]=0; variables[1]=1;
//  vector<vector<size_t> > variablePositions(3);
//  variablePositions[0].resize(1); variablePositions[0][0]=0;
//  variablePositions[1].resize(2); variablePositions[1][0]=0; variablePositions[1][1]=1;
//  variablePositions[2].resize(1); variablePositions[2][0]=0;
//
//  JacobianFactor actual = *JacobianFactor::Combine(gfg, varindex, factors, variables, variablePositions);
//
//  Matrix zero3x3 = zeros(3,3);
//  Matrix A0 = gtsam::stack(3, &A00, &A10, &A20);
//  Matrix A1 = gtsam::stack(3, &zero3x3, &A11, &zero3x3);
//  Vector b = gtsam::concatVectors(3, &b0, &b1, &b2);
//  Vector sigmas = gtsam::concatVectors(3, &s0, &s1, &s2);
//
//  JacobianFactor expected(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));
//
//  EXPECT(assert_equal(expected, actual));
//}

/* ************************************************************************* */
TEST(GaussianFactor, Combine2)
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

  JacobianFactor actual = *JacobianFactor::Combine(gfg, VariableSlots(gfg));

  Matrix zero3x3 = zeros(3,3);
  Matrix A0 = gtsam::stack(3, &A10, &zero3x3, &zero3x3);
  Matrix A1 = gtsam::stack(3, &A11, &A01, &A21);
  Vector b = gtsam::concatVectors(3, &b1, &b0, &b2);
  Vector sigmas = gtsam::concatVectors(3, &s1, &s0, &s2);

  JacobianFactor expected(0, A0, 1, A1, b, noiseModel::Diagonal::Sigmas(sigmas, true));

  EXPECT(assert_equal(expected, actual));
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

  pair<GaussianBayesNet::shared_ptr, JacobianFactor::shared_ptr> actualQR(JacobianFactor::CombineAndEliminate(
      *gfg.dynamicCastFactors<FactorGraph<JacobianFactor> >(), 1));

  EXPECT(assert_equal(expectedBN, *actualQR.first));
  EXPECT(assert_equal(expectedFactor, *actualQR.second));
}

///* ************************************************************************* */
//TEST(GaussianFactor, operators )
//{
//	Matrix I = eye(2);
//	Vector b = Vector_(2,0.2,-0.1);
//	JacobianFactor lf(_x1_, -I, _x2_, I, b, sigma0_1);
//
//	VectorValues c;
//	c.insert(_x1_,Vector_(2,10.,20.));
//	c.insert(_x2_,Vector_(2,30.,60.));
//
//	// test A*x
//	Vector expectedE = Vector_(2,200.,400.), e = lf*c;
//	EXPECT(assert_equal(expectedE,e));
//
//	// test A^e
//	VectorValues expectedX;
//	expectedX.insert(_x1_,Vector_(2,-2000.,-4000.));
//	expectedX.insert(_x2_,Vector_(2, 2000., 4000.));
//	EXPECT(assert_equal(expectedX,lf^e));
//
//	// test transposeMultiplyAdd
//	VectorValues x;
//	x.insert(_x1_,Vector_(2, 1.,2.));
//	x.insert(_x2_,Vector_(2, 3.,4.));
//	VectorValues expectedX2 = x + 0.1 * (lf^e);
//	lf.transposeMultiplyAdd(0.1,e,x);
//	EXPECT(assert_equal(expectedX2,x));
//}

///* ************************************************************************* */
//TEST( NonlinearFactorGraph, combine2){
//	double sigma1 = 0.0957;
//	Matrix A11(2,2);
//	A11(0,0) = 1; A11(0,1) =  0;
//	A11(1,0) = 0;       A11(1,1) = 1;
//	Vector b(2);
//	b(0) = 2; b(1) = -1;
//	JacobianFactor::shared_ptr f1(new JacobianFactor(_x1_, A11, b*sigma1, sharedSigma(2,sigma1)));
//
//	double sigma2 = 0.5;
//	A11(0,0) = 1; A11(0,1) =  0;
//	A11(1,0) = 0; A11(1,1) = -1;
//	b(0) = 4 ; b(1) = -5;
//	JacobianFactor::shared_ptr f2(new JacobianFactor(_x1_, A11, b*sigma2, sharedSigma(2,sigma2)));
//
//	double sigma3 = 0.25;
//	A11(0,0) = 1; A11(0,1) =  0;
//	A11(1,0) = 0; A11(1,1) = -1;
//	b(0) = 3 ; b(1) = -88;
//	JacobianFactor::shared_ptr f3(new JacobianFactor(_x1_, A11, b*sigma3, sharedSigma(2,sigma3)));
//
//	// TODO: find a real sigma value for this example
//	double sigma4 = 0.1;
//	A11(0,0) = 6; A11(0,1) =  0;
//	A11(1,0) = 0; A11(1,1) = 7;
//	b(0) = 5 ; b(1) = -6;
//	JacobianFactor::shared_ptr f4(new JacobianFactor(_x1_, A11*sigma4, b*sigma4, sharedSigma(2,sigma4)));
//
//	vector<JacobianFactor::shared_ptr> lfg;
//	lfg.push_back(f1);
//	lfg.push_back(f2);
//	lfg.push_back(f3);
//	lfg.push_back(f4);
//	JacobianFactor combined(lfg);
//
//	Vector sigmas = Vector_(8, sigma1, sigma1, sigma2, sigma2, sigma3, sigma3, sigma4, sigma4);
//	Matrix A22(8,2);
//	A22(0,0) = 1;   A22(0,1) =  0;
//	A22(1,0) = 0;   A22(1,1) = 1;
//	A22(2,0) = 1;   A22(2,1) =  0;
//	A22(3,0) = 0;   A22(3,1) = -1;
//	A22(4,0) = 1;   A22(4,1) =  0;
//	A22(5,0) = 0;   A22(5,1) = -1;
//	A22(6,0) = 0.6; A22(6,1) =  0;
//	A22(7,0) = 0;   A22(7,1) =  0.7;
//	Vector exb(8);
//	exb(0) = 2*sigma1 ; exb(1) = -1*sigma1;  exb(2) = 4*sigma2 ; exb(3) = -5*sigma2;
//	exb(4) = 3*sigma3 ; exb(5) = -88*sigma3; exb(6) = 5*sigma4 ; exb(7) = -6*sigma4;
//
//	vector<pair<Index, Matrix> > meas;
//	meas.push_back(make_pair(_x1_, A22));
//	JacobianFactor expected(meas, exb, sigmas);
//	EXPECT(assert_equal(expected,combined));
//}
//
///* ************************************************************************* */
//TEST(GaussianFactor, linearFactorN){
//	Matrix I = eye(2);
//  vector<JacobianFactor::shared_ptr> f;
//  SharedDiagonal model = sharedSigma(2,1.0);
//  f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x1_, I, Vector_(2,
//			10.0, 5.0), model)));
//	f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x1_, -10 * I,
//			_x2_, 10 * I, Vector_(2, 1.0, -2.0), model)));
//	f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x2_, -10 * I,
//			_x3_, 10 * I, Vector_(2, 1.5, -1.5), model)));
//	f.push_back(JacobianFactor::shared_ptr(new JacobianFactor(_x3_, -10 * I,
//			_x4_, 10 * I, Vector_(2, 2.0, -1.0), model)));
//
//  JacobianFactor combinedFactor(f);
//
//  vector<pair<Index, Matrix> > combinedMeasurement;
//  combinedMeasurement.push_back(make_pair(_x1_, Matrix_(8,2,
//      1.0,  0.0,
//      0.0,  1.0,
//    -10.0,  0.0,
//      0.0,-10.0,
//      0.0,  0.0,
//      0.0,  0.0,
//      0.0,  0.0,
//      0.0,  0.0)));
//  combinedMeasurement.push_back(make_pair(_x2_, Matrix_(8,2,
//      0.0,  0.0,
//      0.0,  0.0,
//     10.0,  0.0,
//      0.0, 10.0,
//    -10.0,  0.0,
//      0.0,-10.0,
//      0.0,  0.0,
//      0.0,  0.0)));
//  combinedMeasurement.push_back(make_pair(_x3_, Matrix_(8,2,
//      0.0,  0.0,
//      0.0,  0.0,
//      0.0,  0.0,
//      0.0,  0.0,
//     10.0,  0.0,
//      0.0, 10.0,
//    -10.0,  0.0,
//      0.0,-10.0)));
//  combinedMeasurement.push_back(make_pair(_x4_, Matrix_(8,2,
//      0.0, 0.0,
//      0.0, 0.0,
//      0.0, 0.0,
//      0.0, 0.0,
//      0.0, 0.0,
//      0.0, 0.0,
//     10.0, 0.0,
//      0.0,10.0)));
//  Vector b = Vector_(8,
//      10.0, 5.0, 1.0, -2.0, 1.5, -1.5, 2.0, -1.0);
//
//  Vector sigmas = repeat(8,1.0);
//  JacobianFactor expected(combinedMeasurement, b, sigmas);
//  EXPECT(assert_equal(expected,combinedFactor));
//}

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
	meas.push_back(make_pair(_x2_, Ax2));
	meas.push_back(make_pair(_l11_, Al1x1));
	JacobianFactor combined(meas, b2, sigmas);

	// eliminate the combined factor
	GaussianConditional::shared_ptr actualCG_QR;
	JacobianFactor::shared_ptr actualLF_QR(new JacobianFactor(combined));
	actualCG_QR = actualLF_QR->eliminateFirst();

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
	GaussianConditional expectedCG(_x2_,d,R11,_l11_,S12,ones(2));
	EXPECT(assert_equal(expectedCG,*actualCG_QR,1e-4));

	// the expected linear factor
	double sigma = 0.2236;
	Matrix Bl1x1 = Matrix_(2,4,
			// l1          x1
			1.00, 0.00, -1.00,  0.00,
			0.00, 1.00, +0.00, -1.00
	)/sigma;
	Vector b1 = Vector_(2,0.0,0.894427);
	JacobianFactor expectedLF(_l11_, Bl1x1, b1, repeat(2,1.0));
	EXPECT(assert_equal(expectedLF,*actualLF_QR,1e-3));
}

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
      make_pair(3, Matrix(ublas::project(Ab, ublas::range(0,4), ublas::range(0,2)))),
      make_pair(5, ublas::project(Ab, ublas::range(0,4), ublas::range(2,4))),
      make_pair(7, ublas::project(Ab, ublas::range(0,4), ublas::range(4,6))),
      make_pair(9, ublas::project(Ab, ublas::range(0,4), ublas::range(6,8))),
      make_pair(11, ublas::project(Ab, ublas::range(0,4), ublas::range(8,10)));
  Vector b1 = ublas::project(ublas::column(Ab, 10), ublas::range(0,4));
  JacobianFactor::shared_ptr factor1(new JacobianFactor(terms1, b1, sharedSigma(4, 0.5)));

  // Create second factor
  list<pair<Index, Matrix> > terms2;
  terms2 +=
      make_pair(5, ublas::project(Ab, ublas::range(4,8), ublas::range(2,4))),
      make_pair(7, ublas::project(Ab, ublas::range(4,8), ublas::range(4,6))),
      make_pair(9, ublas::project(Ab, ublas::range(4,8), ublas::range(6,8))),
      make_pair(11, ublas::project(Ab, ublas::range(4,8), ublas::range(8,10)));
  Vector b2 = ublas::project(ublas::column(Ab, 10), ublas::range(4,8));
  JacobianFactor::shared_ptr factor2(new JacobianFactor(terms2, b2, sharedSigma(4, 0.5)));

  // Create third factor
  list<pair<Index, Matrix> > terms3;
  terms3 +=
      make_pair(7, ublas::project(Ab, ublas::range(8,12), ublas::range(4,6))),
      make_pair(9, ublas::project(Ab, ublas::range(8,12), ublas::range(6,8))),
      make_pair(11, ublas::project(Ab, ublas::range(8,12), ublas::range(8,10)));
  Vector b3 = ublas::project(ublas::column(Ab, 10), ublas::range(8,12));
  JacobianFactor::shared_ptr factor3(new JacobianFactor(terms3, b3, sharedSigma(4, 0.5)));

  // Create fourth factor
  list<pair<Index, Matrix> > terms4;
  terms4 +=
      make_pair(11, ublas::project(Ab, ublas::range(12,14), ublas::range(8,10)));
  Vector b4 = ublas::project(ublas::column(Ab, 10), ublas::range(12,14));
  JacobianFactor::shared_ptr factor4(new JacobianFactor(terms4, b4, sharedSigma(2, 0.5)));

  // Create factor graph
  GaussianFactorGraph factors;
  factors.push_back(factor1);
  factors.push_back(factor2);
  factors.push_back(factor3);
  factors.push_back(factor4);

  // Create combined factor
  JacobianFactor combined(*JacobianFactor::Combine(factors, VariableSlots(factors)));

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
  Matrix R1 = ublas::project(R, ublas::range(0,2), ublas::range(0,2));
  list<pair<Index, Matrix> > cterms1;
  cterms1 +=
      make_pair(5, ublas::project(R, ublas::range(0,2), ublas::range(2,4))),
      make_pair(7, ublas::project(R, ublas::range(0,2), ublas::range(4,6))),
      make_pair(9, ublas::project(R, ublas::range(0,2), ublas::range(6,8))),
      make_pair(11, ublas::project(R, ublas::range(0,2), ublas::range(8,10)));
  Vector d1 = ublas::project(ublas::column(R, 10), ublas::range(0,2));
  GaussianConditional::shared_ptr cond1(new GaussianConditional(3, d1, R1, cterms1, ones(2)));

  // Expected conditional on second variable from next 2 rows of R
  Matrix R2 = ublas::project(R, ublas::range(2,4), ublas::range(2,4));
  list<pair<Index, Matrix> > cterms2;
  cterms2 +=
      make_pair(7, ublas::project(R, ublas::range(2,4), ublas::range(4,6))),
      make_pair(9, ublas::project(R, ublas::range(2,4), ublas::range(6,8))),
      make_pair(11, ublas::project(R, ublas::range(2,4), ublas::range(8,10)));
  Vector d2 = ublas::project(ublas::column(R, 10), ublas::range(2,4));
  GaussianConditional::shared_ptr cond2(new GaussianConditional(5, d2, R2, cterms2, ones(2)));

  // Expected conditional on third variable from next 2 rows of R
  Matrix R3 = ublas::project(R, ublas::range(4,6), ublas::range(4,6));
  list<pair<Index, Matrix> > cterms3;
  cterms3 +=
      make_pair(9, ublas::project(R, ublas::range(4,6), ublas::range(6,8))),
      make_pair(11, ublas::project(R, ublas::range(4,6), ublas::range(8,10)));
  Vector d3 = ublas::project(ublas::column(R, 10), ublas::range(4,6));
  GaussianConditional::shared_ptr cond3(new GaussianConditional(7, d3, R3, cterms3, ones(2)));

  // Create expected Bayes net fragment from three conditionals above
  GaussianBayesNet expectedFragment;
  expectedFragment.push_back(cond1);
  expectedFragment.push_back(cond2);
  expectedFragment.push_back(cond3);

  // Get expected matrices for remaining factor
  Matrix Ae1 = ublas::project(R, ublas::range(6,10), ublas::range(6,8));
  Matrix Ae2 = ublas::project(R, ublas::range(6,10), ublas::range(8,10));
  Vector be = ublas::project(ublas::column(R, 10), ublas::range(6,10));

  // Eliminate (3 frontal variables, 6 scalar columns) using QR !!!!
  GaussianBayesNet actualFragment_QR = *actualFactor_QR.eliminate(3);
  EXPECT(assert_equal(expectedFragment, actualFragment_QR, 0.001));
  EXPECT(assert_equal(size_t(2), actualFactor_QR.keys().size()));
  EXPECT(assert_equal(Index(9), actualFactor_QR.keys()[0]));
  EXPECT(assert_equal(Index(11), actualFactor_QR.keys()[1]));
  EXPECT(assert_equal(Ae1, actualFactor_QR.getA(actualFactor_QR.begin()), 0.001));
  EXPECT(assert_equal(Ae2, actualFactor_QR.getA(actualFactor_QR.begin()+1), 0.001));
  EXPECT(assert_equal(be, actualFactor_QR.getb(), 0.001));
  EXPECT(assert_equal(ones(4), actualFactor_QR.get_model()->sigmas(), 0.001));

  // Eliminate (3 frontal variables, 6 scalar columns) using Cholesky !!!!
//  GaussianBayesNet actualFragment_Chol = *actualFactor_Chol.eliminate(3, JacobianFactor::SOLVE_CHOLESKY);
//  EXPECT(assert_equal(expectedFragment, actualFragment_Chol, 0.001));
//  EXPECT(assert_equal(size_t(2), actualFactor_Chol.keys().size()));
//  EXPECT(assert_equal(Index(9), actualFactor_Chol.keys()[0]));
//  EXPECT(assert_equal(Index(11), actualFactor_Chol.keys()[1]));
//  EXPECT(assert_equal(Ae1, actualFactor_Chol.getA(actualFactor_Chol.begin()), 0.001)); ////
//  EXPECT(linear_dependent(Ae2, actualFactor_Chol.getA(actualFactor_Chol.begin()+1), 0.001));
//  EXPECT(assert_equal(be, actualFactor_Chol.getb(), 0.001)); ////
//  EXPECT(assert_equal(ones(4), actualFactor_Chol.get_sigmas(), 0.001));
}

/* ************************************************************************* */
TEST(GaussianFactor, default_error )
{
	JacobianFactor f;
	vector<size_t> dims;
	VectorValues c(dims);
	double actual = f.error(c);
	EXPECT(actual==0.0);
}

////* ************************************************************************* */
//TEST(GaussianFactor, eliminate_empty )
//{
//	// create an empty factor
//	JacobianFactor f;
//
//	// eliminate the empty factor
//	GaussianConditional::shared_ptr actualCG;
//	JacobianFactor::shared_ptr actualLF(new JacobianFactor(f));
//	actualCG = actualLF->eliminateFirst();
//
//	// expected Conditional Gaussian is just a parent-less node with P(x)=1
//	GaussianConditional expectedCG(_x2_);
//
//	// expected remaining factor is still empty :-)
//	JacobianFactor expectedLF;
//
//	// check if the result matches
//	EXPECT(actualCG->equals(expectedCG));
//	EXPECT(actualLF->equals(expectedLF));
//}

//* ************************************************************************* */
TEST(GaussianFactor, empty )
{
	// create an empty factor
	JacobianFactor f;
	EXPECT(f.empty()==true);
}

/* ************************************************************************* */
// small aux. function to print out lists of anything
template<class T>
void print(const list<T>& i) {
	copy(i.begin(), i.end(), ostream_iterator<T> (cout, ","));
	cout << endl;
}

///* ************************************************************************* */
//TEST(GaussianFactor, tally_separator )
//{
//	JacobianFactor f(_x1_, eye(2), _x2_, eye(2), _l1_, eye(2), ones(2), sigma0_1);
//
//	std::set<Index> act1, act2, act3;
//	f.tally_separator(_x1_,	act1);
//	f.tally_separator(_x2_,	act2);
//	f.tally_separator(_l1_,	act3);
//
//	EXPECT(act1.size() == 2);
//	EXPECT(act1.count(_x2_) == 1);
//	EXPECT(act1.count(_l1_) == 1);
//
//	EXPECT(act2.size() == 2);
//	EXPECT(act2.count(_x1_) == 1);
//	EXPECT(act2.count(_l1_) == 1);
//
//	EXPECT(act3.size() == 2);
//	EXPECT(act3.count(_x1_) == 1);
//	EXPECT(act3.count(_x2_) == 1);
//}

/* ************************************************************************* */
TEST(GaussianFactor, CONSTRUCTOR_GaussianConditional )
{
	Matrix R11 = eye(2);
	Matrix S12 = Matrix_(2,2,
			-0.200001, 0.00,
			+0.00,-0.200001
	);
	Vector d(2); d(0) = 2.23607; d(1) = -1.56525;
	Vector sigmas =repeat(2,0.29907);
	GaussianConditional::shared_ptr CG(new GaussianConditional(_x2_,d,R11,_l11_,S12,sigmas));

	// Call the constructor we are testing !
	JacobianFactor actualLF(*CG);

	JacobianFactor expectedLF(_x2_,R11,_l11_,S12,d, sigmas);
	EXPECT(assert_equal(expectedLF,actualLF,1e-5));
}

///* ************************************************************************* */
//TEST(GaussianFactor, CONSTRUCTOR_GaussianConditionalConstrained )
//{
//  Matrix Ax = eye(2);
//  Vector b = Vector_(2, 3.0, 5.0);
//  SharedDiagonal noisemodel = noiseModel::Constrained::All(2);
//  JacobianFactor::shared_ptr expected(new JacobianFactor(_x0_, Ax, b, noisemodel));
//  GaussianFactorGraph graph;
//  graph.push_back(expected);
//
//  GaussianConditional::shared_ptr conditional = GaussianSequentialSolver::EliminateUntil(graph,_x0_+1);
//  JacobianFactor actual(*conditional);
//
//  EXPECT(assert_equal(*expected, actual));
//}

/* ************************************************************************* */
TEST ( JacobianFactor, constraint_eliminate1 )
{
	// construct a linear constraint
	Vector v(2); v(0)=1.2; v(1)=3.4;
	Index key = _x0_;
	JacobianFactor lc(key, eye(2), v, constraintModel);

	// eliminate it
	GaussianConditional::shared_ptr actualCG;
	JacobianFactor::shared_ptr actualLF(new JacobianFactor(lc));
	actualCG = actualLF->eliminateFirst();

	// verify linear factor
	EXPECT(actualLF->size() == 0);

	// verify conditional Gaussian
	Vector sigmas = Vector_(2, 0.0, 0.0);
	GaussianConditional expCG(_x0_, v, eye(2), sigmas);
	EXPECT(assert_equal(expCG, *actualCG));
}

/* ************************************************************************* */
TEST ( JacobianFactor, constraint_eliminate2 )
{
	// Construct a linear constraint
	// RHS
	Vector b(2); b(0)=3.0; b(1)=4.0;

	// A1 - invertible
	Matrix A1(2,2);
	A1(0,0) = 1.0 ; A1(0,1) = 2.0;
	A1(1,0) = 2.0 ; A1(1,1) = 1.0;

	// A2 - not invertible
	Matrix A2(2,2);
	A2(0,0) = 1.0 ; A2(0,1) = 2.0;
	A2(1,0) = 2.0 ; A2(1,1) = 4.0;

	JacobianFactor lc(_x_, A1, _y_, A2, b, constraintModel);

	// eliminate x and verify results
	GaussianConditional::shared_ptr actualCG;
	JacobianFactor::shared_ptr actualLF(new JacobianFactor(lc));
	actualCG = actualLF->eliminateFirst();

	// LF should be null
	JacobianFactor expectedLF;
	EXPECT(assert_equal(*actualLF, expectedLF));

	// verify CG
	Matrix R = Matrix_(2, 2,
			1.0,    2.0,
			0.0,    1.0);
	Matrix S = Matrix_(2,2,
			1.0,    2.0,
			0.0,    0.0);
	Vector d = Vector_(2, 3.0, 0.6666);
	GaussianConditional expectedCG(_x_, d, R, _y_, S, zero(2));
	EXPECT(assert_equal(expectedCG, *actualCG, 1e-4));
}

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

  JacobianFactor actual(1, A1, 3, A2, 5, A3, b, sharedSigma(2, 1.0));
  GaussianFactorGraph actualFG; actualFG.push_back(JacobianFactor::shared_ptr(new JacobianFactor(actual)));
  VariableIndex actualIndex(actualFG);
  actual.permuteWithInverse(inversePermutation);
//  actualIndex.permute(*inversePermutation.inverse());

  JacobianFactor expected(0, A3, 2, A2, 4, A1, b, sharedSigma(2, 1.0));
  GaussianFactorGraph expectedFG; expectedFG.push_back(JacobianFactor::shared_ptr(new JacobianFactor(expected)));
//  GaussianVariableIndex expectedIndex(expectedFG);

  EXPECT(assert_equal(expected, actual));

//  // todo: fix this!!!  VariableIndex should not hold slots
//  for(Index j=0; j<actualIndex.size(); ++j) {
//    BOOST_FOREACH( GaussianVariableIndex::mapped_factor_type& factor_pos, actualIndex[j]) {
//      factor_pos.variablePosition = numeric_limits<Index>::max(); }
//  }
//  for(Index j=0; j<expectedIndex.size(); ++j) {
//    BOOST_FOREACH( GaussianVariableIndex::mapped_factor_type& factor_pos, expectedIndex[j]) {
//      factor_pos.variablePosition = numeric_limits<Index>::max(); }
//  }
//  EXPECT(assert_equal(expectedIndex, actualIndex));
}

///* ************************************************************************* */
//TEST(GaussianFactor, erase)
//{
//	Vector b = Vector_(3, 1., 2., 3.);
//	SharedDiagonal noise = noiseModel::Diagonal::Sigmas(Vector_(3,1.,1.,1.));
//	std::list<std::pair<Index, Matrix> > terms;
//	terms.push_back(make_pair(_x0_, eye(2)));
//	terms.push_back(make_pair(_x1_, 2.*eye(2)));
//
//	JacobianFactor actual(terms, b, noise);
//	int erased = actual.erase_A(_x0_);
//
//	LONGS_EQUAL(1, erased);
//	JacobianFactor expected(_x1_, 2.*eye(2), b, noise);
//	EXPECT(assert_equal(expected, actual));
//}

///* ************************************************************************* */
//TEST(GaussianFactor, eliminateMatrix)
//{
//	Matrix Ab = Matrix_(3, 4,
//			1., 2., 0., 3.,
//			0., 4., 5., 6.,
//			0., 0., 7., 8.);
//	SharedDiagonal model(Vector_(3, 0.5, 0.5, 0.5));
//	Ordering frontals; frontals += _x1_, _x2_;
//	Ordering separator; separator += _x3_;
//	Dimensions dimensions;
//	dimensions.insert(make_pair(_x1_, 1));
//	dimensions.insert(make_pair(_x2_, 1));
//	dimensions.insert(make_pair(_x3_, 1));
//
//	JacobianFactor::shared_ptr factor;
//	GaussianBayesNet bn;
//	boost::tie(bn, factor) =
//			JacobianFactor::eliminateMatrix(Ab, NULL, model, frontals, separator, dimensions);
//
//	GaussianBayesNet bn_expected;
//	GaussianBayesNet::sharedConditional conditional1(new GaussianConditional(_x1_, Vector_(1, 6.), Matrix_(1, 1, 2.),
//			_x2_, Matrix_(1, 1, 4.), _x3_, Matrix_(1, 1, 0.), Vector_(1, 1.)));
//	GaussianBayesNet::sharedConditional conditional2(new GaussianConditional(_x2_, Vector_(1, 12.), Matrix_(1, 1, 8.),
//			_x3_, Matrix_(1, 1, 10.), Vector_(1, 1.)));
//	bn_expected.push_back(conditional1);
//	bn_expected.push_back(conditional2);
//	EXPECT(assert_equal(bn_expected, bn));
//
//	JacobianFactor factor_expected(_x3_, Matrix_(1, 1, 14.), Vector_(1, 16.), SharedDiagonal(Vector_(1, 1.)));
//	EXPECT(assert_equal(factor_expected, *factor));
//}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
