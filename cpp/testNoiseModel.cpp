/*
 * testNoiseModel.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <iostream>
#include "NoiseModel.h"
#include "SharedGaussian.h"
#include "SharedDiagonal.h"

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
	m.push_back(Gaussian::Information(Q));
	m.push_back(Diagonal::Sigmas(Vector_(3, sigma, sigma, sigma)));
	m.push_back(Diagonal::Variances(Vector_(3, var, var, var)));
	m.push_back(Diagonal::Precisions(Vector_(3, prc, prc, prc)));
	m.push_back(Isotropic::Sigma(3, sigma));
	m.push_back(Isotropic::Variance(3, var));
	m.push_back(Isotropic::Precision(3, prc));

	// test whiten
	BOOST_FOREACH(Gaussian::shared_ptr mi, m)
		CHECK(assert_equal(whitened,mi->whiten(unwhitened)));

	// test unwhiten
	BOOST_FOREACH(Gaussian::shared_ptr mi, m)
		CHECK(assert_equal(unwhitened,mi->unwhiten(whitened)));

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
		CHECK(assert_equal(expectedR,mi->R()));

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
		CHECK(assert_equal(expected,mi->Whiten(H)));

	// can only test inplace version once :-)
	m[0]->WhitenInPlace(H);
	CHECK(assert_equal(expected,H));
}

/* ************************************************************************* */
TEST(NoiseModel, Unit) {
	Vector v = Vector_(3,5.0,10.0,15.0);
	Gaussian::shared_ptr u(Unit::Create(3));
	CHECK(assert_equal(v,u->whiten(v)));
}

/* ************************************************************************* */
TEST(NoiseModel, equals)
{
	Gaussian::shared_ptr g = Gaussian::SqrtInformation(R);
	Diagonal::shared_ptr d = Diagonal::Sigmas(Vector_(3, sigma, sigma, sigma));
	Isotropic::shared_ptr i = Isotropic::Sigma(3, sigma);
	CHECK(assert_equal(*g,*g));
}

/* ************************************************************************* */
TEST(NoiseModel, ConstrainedMixed )
{
	Vector feasible = Vector_(3, 1.0, 0.0, 1.0),
			infeasible = Vector_(3, 1.0, 1.0, 1.0);
	Constrained::shared_ptr d = Constrained::MixedSigmas(Vector_(3, sigma, 0.0, sigma));
	CHECK(assert_equal(Vector_(3, 0.5, inf, 0.5),d->whiten(infeasible)));
	CHECK(assert_equal(Vector_(3, 0.5, 0.0, 0.5),d->whiten(feasible)));
	DOUBLES_EQUAL(inf,d->Mahalanobis(infeasible),1e-9);
	DOUBLES_EQUAL(0.5,d->Mahalanobis(feasible),1e-9);
}

/* ************************************************************************* */
TEST(NoiseModel, ConstrainedAll )
{
	Vector feasible = Vector_(3, 0.0, 0.0, 0.0),
			infeasible = Vector_(3, 1.0, 1.0, 1.0);

	Constrained::shared_ptr i = Constrained::All(3);
	CHECK(assert_equal(Vector_(3, inf, inf, inf),i->whiten(infeasible)));
	CHECK(assert_equal(Vector_(3, 0.0, 0.0, 0.0),i->whiten(feasible)));
	DOUBLES_EQUAL(inf,i->Mahalanobis(infeasible),1e-9);
	DOUBLES_EQUAL(0.0,i->Mahalanobis(feasible),1e-9);
}

/* ************************************************************************* */
TEST( NoiseModel, QR )
{
	// create a matrix to eliminate
	Matrix Ab1 = Matrix_(4, 6+1,
		   -1.,  0.,  1.,  0.,  0.,  0., -0.2,
			0., -1.,  0.,  1.,  0.,  0.,  0.3,
			1.,  0.,  0.,  0., -1.,  0.,  0.2,
			0.,  1.,  0.,  0.,  0., -1., -0.1);
	Matrix Ab2 = Ab1; // otherwise overwritten !
	Vector sigmas = Vector_(4, 0.2, 0.2, 0.1, 0.1);

	// Expected result
	Vector expectedSigmas = Vector_(4, 0.0894427, 0.0894427, 0.223607, 0.223607);
	SharedDiagonal expectedModel = noiseModel::Diagonal::Sigmas(expectedSigmas);

	// Call Gaussian version
	SharedDiagonal diagonal = noiseModel::Diagonal::Sigmas(sigmas);
	SharedDiagonal actual1 = diagonal->QR(Ab1);
	SharedDiagonal expected = noiseModel::Unit::Create(4);
	CHECK(assert_equal(*expected,*actual1));
	Matrix expectedRd1 = Matrix_(4, 6+1,
			11.1803,   0.0,   -2.23607, 0.0,    -8.94427, 0.0,     2.23607,
			0.0,   11.1803,    0.0,    -2.23607, 0.0,    -8.94427,-1.56525,
			0.0,       0.0,    4.47214, 0.0,    -4.47214, 0.0,     0.0,
			0.0,       0.0,   0.0,     4.47214, 0.0,    -4.47214, 0.894427);
	CHECK(assert_equal(expectedRd1,Ab1,1e-4)); // Ab was modified in place !!!

	// Call Constrained version
	SharedDiagonal constrained = noiseModel::Constrained::MixedSigmas(sigmas);
	SharedDiagonal actual2 = constrained->QR(Ab2);
	SharedDiagonal expectedModel2 = noiseModel::Diagonal::Sigmas(expectedSigmas);
	CHECK(assert_equal(*expectedModel2,*actual2));
	Matrix expectedRd2 = Matrix_(4, 6+1,
			1.,  0., -0.2,  0., -0.8, 0.,  0.2,
			0.,  1.,  0.,-0.2,   0., -0.8,-0.14,
			0.,  0.,  1.,   0., -1.,  0.,  0.0,
			0.,  0.,  0.,   1.,  0., -1.,  0.2);
	CHECK(assert_equal(expectedRd2,Ab2,1e-6)); // Ab was modified in place !!!
}

/* ************************************************************************* */
TEST(NoiseModel, QRNan )
{
	SharedDiagonal constrained = noiseModel::Constrained::All(2);
	Matrix Ab = Matrix_(2, 5, 1., 2., 1., 2., 3., 2., 1., 2., 4., 4.);

	SharedDiagonal expected = noiseModel::Constrained::All(2);
	Matrix expectedAb = Matrix_(2, 5, 1., 2., 1., 2., 3., 0., 1., 0., 0., 2.0/3);

	SharedDiagonal actual = constrained->QR(Ab);
	CHECK(assert_equal(*expected,*actual));
	CHECK(assert_equal(expectedAb,Ab));
}

/* ************************************************************************* */
TEST(NoiseModel, SmartCovariance )
{
	bool smart = true;
	SharedGaussian expected = Unit::Create(3);
	SharedGaussian actual = Gaussian::Covariance(eye(3), smart);
	CHECK(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, ScalarOrVector )
{
	bool smart = true;
	SharedGaussian expected = Unit::Create(3);
	SharedGaussian actual = Gaussian::Covariance(eye(3), smart);
	CHECK(assert_equal(*expected,*actual));
}

/* ************************************************************************* */
TEST(NoiseModel, WhitenInPlace)
{
	Vector sigmas = Vector_(3, 0.1, 0.1, 0.1);
	SharedDiagonal model(sigmas);
	Matrix A = eye(3);
	model->WhitenInPlace(A);
	Matrix expected = eye(3) * 10;
	CHECK(assert_equal(expected, A));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
