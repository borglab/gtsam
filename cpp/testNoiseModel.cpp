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

/* ************************************************************************* */
TEST(NoiseModel, constructors)
{
	double sigma = 2, s_1=1.0/sigma, var = sigma*sigma, prc = 1.0/var;
	Vector whitened = Vector_(3,5.0,10.0,15.0);
	Vector unwhitened = Vector_(3,10.0,20.0,30.0);

	// Construct noise models
	vector<GaussianNoiseModel::shared_ptr> m;
	m.push_back(GaussianNoiseModel::SqrtInformation(Matrix_(3, 3,
			s_1, 0.0, 0.0,
			0.0, s_1, 0.0,
			0.0, 0.0, s_1)));
	m.push_back(GaussianNoiseModel::Covariance(Matrix_(3, 3,
			var, 0.0, 0.0,
			0.0, var, 0.0,
			0.0, 0.0, var)));
	m.push_back(GaussianNoiseModel::Information(Matrix_(3, 3,
			prc, 0.0, 0.0,
			0.0, prc, 0.0,
			0.0, 0.0, prc)));
	m.push_back(Diagonal::Sigmas(Vector_(3, sigma, sigma, sigma)));
	m.push_back(Diagonal::Variances(Vector_(3, var, var, var)));
	m.push_back(Diagonal::Precisions(Vector_(3, prc, prc, prc)));
	m.push_back(Isotropic::Sigma(3, sigma));
	m.push_back(Isotropic::Variance(3, var));
	m.push_back(Isotropic::Precision(3, prc));

	// test whiten
	int i=0;
	BOOST_FOREACH(GaussianNoiseModel::shared_ptr mi, m)
		CHECK(assert_equal(whitened,mi->whiten(unwhitened)));

	// test unwhiten
	BOOST_FOREACH(GaussianNoiseModel::shared_ptr mi, m)
		CHECK(assert_equal(unwhitened,mi->unwhiten(whitened)));

	// test R matrix
	Matrix expectedR(Matrix_(3, 3,
			s_1, 0.0, 0.0,
			0.0, s_1, 0.0,
			0.0, 0.0, s_1));

	BOOST_FOREACH(GaussianNoiseModel::shared_ptr mi, m)
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

	BOOST_FOREACH(GaussianNoiseModel::shared_ptr mi, m)
		CHECK(assert_equal(expected,mi->Whiten(H)));

	// can only test inplace version once :-)
	m[0]->WhitenInPlace(H);
	CHECK(assert_equal(expected,H));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
