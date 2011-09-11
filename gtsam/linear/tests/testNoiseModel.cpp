/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testNoiseModel.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */


#include <iostream>
#include <boost/foreach.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/SharedGaussian.h>
#include <gtsam/linear/SharedDiagonal.h>

using namespace std;
using namespace gtsam;
using namespace noiseModel;

static double sigma = 2, s_1=1.0/sigma, var = sigma*sigma, prc = 1.0/var;
static Matrix R = Matrix_(3, 3,
		s_1, 0.0, 0.0,
		0.0, s_1, 0.0,
		0.0, 0.0, s_1);
static Matrix Sigma = Matrix_(3, 3,
		var, 0.0, 0.0,
		0.0, var, 0.0,
		0.0, 0.0, var);
static Matrix Q = Matrix_(3, 3,
		prc, 0.0, 0.0,
		0.0, prc, 0.0,
		0.0, 0.0, prc);

static double inf = std::numeric_limits<double>::infinity();

/* ************************************************************************* */
TEST(NoiseModel, constructors)
{
	Vector whitened = Vector_(3,5.0,10.0,15.0);
	Vector unwhitened = Vector_(3,10.0,20.0,30.0);

	// Construct noise models
	vector<Gaussian::shared_ptr> m;
	m.push_back(Gaussian::SqrtInformation(R));
	m.push_back(Gaussian::Covariance(Sigma));
	//m.push_back(Gaussian::Information(Q));
	m.push_back(Diagonal::Sigmas(Vector_(3, sigma, sigma, sigma)));
	m.push_back(Diagonal::Variances(Vector_(3, var, var, var)));
	m.push_back(Diagonal::Precisions(Vector_(3, prc, prc, prc)));
	m.push_back(Isotropic::Sigma(3, sigma));
	m.push_back(Isotropic::Variance(3, var));
	m.push_back(Isotropic::Precision(3, prc));

	// test whiten
	BOOST_FOREACH(Gaussian::shared_ptr mi, m)
		EXPECT(assert_equal(whitened,mi->whiten(unwhitened)));

	// test unwhiten
	BOOST_FOREACH(Gaussian::shared_ptr mi, m)
		EXPECT(assert_equal(unwhitened,mi->unwhiten(whitened)));

	// test Mahalanobis distance
	double distance = 5*5+10*10+15*15;
	BOOST_FOREACH(Gaussian::shared_ptr mi, m)
		DOUBLES_EQUAL(distance,mi->Mahalanobis(unwhitened),1e-9);

	// test R matrix
	Matrix expectedR(Matrix_(3, 3,
			s_1, 0.0, 0.0,
			0.0, s_1, 0.0,
			0.0, 0.0, s_1));

	BOOST_FOREACH(Gaussian::shared_ptr mi, m)
		EXPECT(assert_equal(expectedR,mi->R()));

	// test Whiten operator
	Matrix H(Matrix_(3, 4,
			0.0, 0.0, 1.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 0.0, 1.0));

	Matrix expected(Matrix_(3, 4,
			0.0, 0.0, s_1, s_1,
			0.0, s_1, 0.0, s_1,
			s_1, 0.0, 0.0, s_1));

	BOOST_FOREACH(Gaussian::shared_ptr mi, m)
		EXPECT(assert_equal(expected,mi->Whiten(H)));

	// can only test inplace version once :-)
	m[0]->WhitenInPlace(H);
	EXPECT(assert_equal(expected,H));
}

/* ************************************************************************* */
TEST(NoiseModel, Unit) {
	Vector v = Vector_(3,5.0,10.0,15.0);
	Gaussian::shared_ptr u(Unit::Create(3));
	EXPECT(assert_equal(v,u->whiten(v)));
}

/* ************************************************************************* */
TEST(NoiseModel, equals)
{
	Gaussian::shared_ptr g1 = Gaussian::SqrtInformation(R),
											 g2 = Gaussian::SqrtInformation(eye(3,3));
	Diagonal::shared_ptr d1 = Diagonal::Sigmas(Vector_(3, sigma, sigma, sigma)),
											 d2 = Diagonal::Sigmas(Vector_(3, 0.1, 0.2, 0.3));
	Isotropic::shared_ptr i1 = Isotropic::Sigma(3, sigma),
												i2 = Isotropic::Sigma(3, 0.7);

	EXPECT(assert_equal(*g1,*g1));
	EXPECT(assert_inequal(*g1, *g2));

	EXPECT(assert_equal(*d1,*d1));
	EXPECT(assert_inequal(*d1,*d2));

	EXPECT(assert_equal(*i1,*i1));
	EXPECT(assert_inequal(*i1,*i2));
}

/* ************************************************************************* */
TEST(NoiseModel, ConstrainedMixed )
{
	Vector feasible = Vector_(3, 1.0, 0.0, 1.0),
			infeasible = Vector_(3, 1.0, 1.0, 1.0);
	Constrained::shared_ptr d = Constrained::MixedSigmas(Vector_(3, sigma, 0.0, sigma));
	EXPECT(assert_equal(Vector_(3, 0.5, inf, 0.5),d->whiten(infeasible)));
	EXPECT(assert_equal(Vector_(3, 0.5, 0.0, 0.5),d->whiten(feasible)));
	DOUBLES_EQUAL(inf,d->Mahalanobis(infeasible),1e-9);
	DOUBLES_EQUAL(0.5,d->Mahalanobis(feasible),1e-9);
}

/* ************************************************************************* */
TEST(NoiseModel, ConstrainedAll )
{
	Vector feasible = Vector_(3, 0.0, 0.0, 0.0),
			infeasible = Vector_(3, 1.0, 1.0, 1.0);

	Constrained::shared_ptr i = Constrained::All(3);
	EXPECT(assert_equal(Vector_(3, inf, inf, inf),i->whiten(infeasible)));
	EXPECT(assert_equal(Vector_(3, 0.0, 0.0, 0.0),i->whiten(feasible)));
	DOUBLES_EQUAL(inf,i->Mahalanobis(infeasible),1e-9);
	DOUBLES_EQUAL(0.0,i->Mahalanobis(feasible),1e-9);
}

/* ************************************************************************* */
namespace exampleQR {
  // create a matrix to eliminate
  Matrix Ab = Matrix_(4, 6+1,
      -1.,  0.,  1.,  0.,  0.,  0., -0.2,
      0., -1.,  0.,  1.,  0.,  0.,  0.3,
      1.,  0.,  0.,  0., -1.,  0.,  0.2,
      0.,  1.,  0.,  0.,  0., -1., -0.1);
  Vector sigmas = Vector_(4, 0.2, 0.2, 0.1, 0.1);

  // the matrix AB yields the following factorized version:
	Matrix Rd = Matrix_(4, 6+1,
			11.1803,   0.0,   -2.23607, 0.0,    -8.94427, 0.0,     2.23607,
			0.0,   11.1803,    0.0,    -2.23607, 0.0,    -8.94427,-1.56525,
			0.0,       0.0,    4.47214, 0.0,    -4.47214, 0.0,     0.0,
			0.0,       0.0,   0.0,     4.47214, 0.0,    -4.47214, 0.894427);

	SharedDiagonal diagonal = noiseModel::Diagonal::Sigmas(sigmas);
}

TEST( NoiseModel, QR )
{
  Matrix Ab1 = exampleQR::Ab;
	Matrix Ab2 = exampleQR::Ab; // otherwise overwritten !

	// Expected result
	Vector expectedSigmas = Vector_(4, 0.0894427, 0.0894427, 0.223607, 0.223607);
	SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(expectedSigmas);

	// Call Gaussian version
	SharedDiagonal actual1 = exampleQR::diagonal->QR(Ab1);
	SharedDiagonal expected = noiseModel::Unit::Create(4);
	EXPECT(assert_equal(*expected,*actual1));
	EXPECT(linear_dependent(exampleQR::Rd,Ab1,1e-4)); // Ab was modified in place !!!

	// Call Constrained version
	SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(exampleQR::sigmas);
	SharedDiagonal actual2 = constrained->QR(Ab2);
	SharedDiagonal expectedModel2 = noiseModel::Diagonal::Sigmas(expectedSigmas);
	EXPECT(assert_equal(*expectedModel2,*actual2,1e-6));
	Matrix expectedRd2 = Matrix_(4, 6+1,
			1.,  0., -0.2,  0., -0.8, 0.,  0.2,
			0.,  1.,  0.,-0.2,   0., -0.8,-0.14,
			0.,  0.,  1.,   0., -1.,  0.,  0.0,
			0.,  0.,  0.,   1.,  0., -1.,  0.2);
	EXPECT(linear_dependent(expectedRd2,Ab2,1e-6)); // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST(NoiseModel, Cholesky)
{
  SharedDiagonal expected = noiseModel::Unit::Create(4);
  Matrix Ab = exampleQR::Ab; // otherwise overwritten !
  SharedDiagonal actual = exampleQR::diagonal->Cholesky(Ab, 4);
  EXPECT(assert_equal(*expected,*actual));
  // Ab was modified in place !!!
  Matrix actualRd = Ab.block(0, 0, actual->dim(), Ab.cols()).triangularView<Eigen::Upper>();
  EXPECT(linear_dependent(exampleQR::Rd,actualRd,1e-4));
}

/* ************************************************************************* */
TEST(NoiseModel, QRNan )
{
	SharedDiagonal constrained = noiseModel::Constrained::All(2);
	Matrix Ab = Matrix_(2, 5, 1., 2., 1., 2., 3., 2., 1., 2., 4., 4.);

	SharedDiagonal expected = noiseModel::Constrained::All(2);
	Matrix expectedAb = Matrix_(2, 5, 1., 2., 1., 2., 3., 0., 1., 0., 0., 2.0/3);

	SharedDiagonal actual = constrained->QR(Ab);
	EXPECT(assert_equal(*expected,*actual));
	EXPECT(assert_equal(expectedAb,Ab));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartCovariance )
{
	bool smart = true;
	SharedGaussian expected = Unit::Create(3);
	SharedGaussian actual = Gaussian::Covariance(eye(3), smart);
	EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, ScalarOrVector )
{
	bool smart = true;
	SharedGaussian expected = Unit::Create(3);
	SharedGaussian actual = Gaussian::Covariance(eye(3), smart);
	EXPECT(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, WhitenInPlace)
{
	Vector sigmas = Vector_(3, 0.1, 0.1, 0.1);
	SharedDiagonal model(sigmas);
	Matrix A = eye(3);
	model->WhitenInPlace(A);
	Matrix expected = eye(3) * 10;
	EXPECT(assert_equal(expected, A));
}

/* ************************************************************************* */
TEST(NoiseModel, robustFunction)
{
  const double k = 5.0, error1 = 1.0, error2 = 10.0;
  const MEstimator::Huber::shared_ptr huber = MEstimator::Huber::Create(k);
  const double weight1 = huber->weight(error1),
               weight2 = huber->weight(error2);
  DOUBLES_EQUAL(1.0, weight1, 1e-8);
  DOUBLES_EQUAL(0.5, weight2, 1e-8);
}

/* ************************************************************************* */
TEST(NoiseModel, robustNoise)
{
  const double k = 10.0, error1 = 1.0, error2 = 100.0;
  Matrix A = Matrix_(2, 2, 1.0, 10.0, 100.0, 1000.0);
  Vector b = Vector_(2, error1, error2);
  const Robust::shared_ptr robust = Robust::Create(
    MEstimator::Huber::Create(k, MEstimator::Huber::Scalar),
    Unit::Create(2));

  robust->WhitenSystem(A,b);

  DOUBLES_EQUAL(error1, b(0), 1e-8);
  DOUBLES_EQUAL(sqrt(k*error2), b(1), 1e-8);

  DOUBLES_EQUAL(1.0, A(0,0), 1e-8);
  DOUBLES_EQUAL(10.0, A(0,1), 1e-8);
  DOUBLES_EQUAL(sqrt(k*100.0), A(1,0), 1e-8);
  DOUBLES_EQUAL(sqrt(k/100.0)*1000.0, A(1,1), 1e-8);
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
