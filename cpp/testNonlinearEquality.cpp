/*
 * @file testNonlinearEquality.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>

#define GTSAM_MAGIC_KEY

#include "VectorConfig.h"
#include "NonlinearEquality.h"

using namespace std;
using namespace gtsam;

typedef NonlinearEquality<VectorConfig,string,Vector> NLE;
typedef boost::shared_ptr<NLE> shared_nle;

bool vector_compare(const Vector& a, const Vector& b) {
	return equal_with_abs_tol(a, b, 1e-5);
}

/* ************************************************************************* */
TEST ( NonlinearEquality, linearization ) {
	Symbol key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	VectorConfig linearize;
	linearize.insert(key, value);

	// create a nonlinear equality constraint
	shared_nle nle(new NLE(key, value,vector_compare));

	// check linearize
	SharedDiagonal constraintModel = noiseModel::Constrained::All(2);
	GaussianFactor expLF(key, eye(2), zero(2), constraintModel);
	GaussianFactor::shared_ptr actualLF = nle->linearize(linearize);
	CHECK(assert_equal(*actualLF, expLF));
}

/* ********************************************************************** */
TEST ( NonlinearEquality, linearization_fail ) {
  Symbol key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	Vector wrong = Vector_(2, 3.0, 4.0);
	VectorConfig bad_linearize;
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_nle nle(new NLE(key, value,vector_compare));

	// check linearize to ensure that it fails for bad linearization points
	try {
		GaussianFactor::shared_ptr actualLF = nle->linearize(bad_linearize);
		CHECK(false);
	} catch (std::invalid_argument) {
		CHECK(true);
	}
}

/* ************************************************************************* */
TEST ( NonlinearEquality, error ) {
  Symbol key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	Vector wrong = Vector_(2, 3.0, 4.0);
	VectorConfig feasible, bad_linearize;
	feasible.insert(key, value);
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_nle nle(new NLE(key, value,vector_compare));

	// check error function outputs
	Vector actual = nle->unwhitenedError(feasible);
	CHECK(assert_equal(actual, zero(2)));

	actual = nle->unwhitenedError(bad_linearize);
	CHECK(assert_equal(actual, repeat(2, 1.0/0.0)));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, equals ) {
	string key1 = "x";
	Vector value1 = Vector_(2, 1.0, 2.0);
	Vector value2 = Vector_(2, 3.0, 4.0);

	// create some constraints to compare
	shared_nle nle1(new NLE(key1, value1,vector_compare));
	shared_nle nle2(new NLE(key1, value1,vector_compare));
	shared_nle nle3(new NLE(key1, value2,vector_compare));

	// verify
	CHECK(nle1->equals(*nle2));  // basic equality = true
	CHECK(nle2->equals(*nle1));  // test symmetry of equals()
	CHECK(!nle1->equals(*nle3)); // test config
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
