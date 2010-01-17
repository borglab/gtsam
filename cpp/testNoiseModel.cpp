/*
 * testNoiseModel.cpp
 *
 *  Created on: Jan 13, 2010
 *      Author: Richard Roberts
 *      Author: Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include <boost/shared_ptr.hpp>
#include <iostream>
#include "NoiseModel.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(NoiseModel, constructors)
{
	double sigma = 2, var = sigma*sigma;
	Vector whitened = Vector_(3,5.0,10.0,15.0);
	Vector unwhitened = Vector_(3,10.0,20.0,30.0);

	// Construct noise models
	Sigma m1(3,sigma);
	Variance m2(3,var);
	Sigmas m3(Vector_(3, sigma, sigma, sigma));
	Variances m4(Vector_(3, var, var, var));
	FullCovariance m5(Matrix_(3, 3,
			var, 0.0, 0.0,
			0.0, var, 0.0,
			0.0, 0.0, var));

	// test whiten
	CHECK(assert_equal(whitened,m1.whiten(unwhitened)));
	CHECK(assert_equal(whitened,m2.whiten(unwhitened)));
	CHECK(assert_equal(whitened,m3.whiten(unwhitened)));
	CHECK(assert_equal(whitened,m4.whiten(unwhitened)));
	CHECK(assert_equal(whitened,m5.whiten(unwhitened)));

	// test unwhiten
	CHECK(assert_equal(unwhitened,m1.unwhiten(whitened)));
	CHECK(assert_equal(unwhitened,m2.unwhiten(whitened)));
	CHECK(assert_equal(unwhitened,m3.unwhiten(whitened)));
	CHECK(assert_equal(unwhitened,m4.unwhiten(whitened)));
	CHECK(assert_equal(unwhitened,m5.unwhiten(whitened)));

	// test R matrix
	double s_1 = 1.0/sigma;
	Matrix expectedR(Matrix_(3, 3,
			s_1, 0.0, 0.0,
			0.0, s_1, 0.0,
			0.0, 0.0, s_1));

	CHECK(assert_equal(expectedR,m1.R()));
	CHECK(assert_equal(expectedR,m2.R()));
	CHECK(assert_equal(expectedR,m3.R()));
	CHECK(assert_equal(expectedR,m4.R()));
	CHECK(assert_equal(expectedR,m5.R()));

	// test Whiten operator
	Matrix H(Matrix_(3, 4,
			0.0, 0.0, 1.0, 1.0,
			0.0, 1.0, 0.0, 1.0,
			1.0, 0.0, 0.0, 1.0));

	Matrix expected(Matrix_(3, 4,
			0.0, 0.0, s_1, s_1,
			0.0, s_1, 0.0, s_1,
			s_1, 0.0, 0.0, s_1));

	CHECK(assert_equal(expected,m1.Whiten(H)));
	CHECK(assert_equal(expected,m2.Whiten(H)));
	CHECK(assert_equal(expected,m3.Whiten(H)));
	CHECK(assert_equal(expected,m4.Whiten(H)));
	CHECK(assert_equal(expected,m5.Whiten(H)));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
