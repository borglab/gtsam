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
	int i=0;
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
	Constrained::shared_ptr d = Constrained::Mixed(Vector_(3, sigma, 0.0, sigma));
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
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
