/*
 * @file testNonlinearEquality.cpp
 * @author Alex Cunningham
 */

#include <CppUnitLite/TestHarness.h>
#include "VectorConfig.h"
#include "NonlinearEquality.h"

using namespace std;
using namespace gtsam;

typedef boost::shared_ptr<NonlinearEquality<VectorConfig> > shared_nle;

bool vector_compare(const std::string& key,
					const VectorConfig& feasible,
					const VectorConfig& input) {
	Vector feas, lin;
	feas = feasible[key];
	lin = input[key];
	return equal_with_abs_tol(lin, feas, 1e-5);
}

/* ************************************************************************* */
TEST ( NonlinearEquality, linearization ) {
	string key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	VectorConfig feasible, linearize;
	feasible.insert(key, value);
	linearize.insert(key, value);

	// create a nonlinear equality constraint
	shared_nle nle(new NonlinearEquality<VectorConfig>(key, feasible, 2, *vector_compare));

	// check linearize
	GaussianFactor expLF(key, eye(2), zero(2), 0.0);
	GaussianFactor::shared_ptr actualLF = nle->linearize(linearize);
	CHECK(assert_equal(*actualLF, expLF));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, linearization_fail ) {
	string key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	Vector wrong = Vector_(2, 3.0, 4.0);
	VectorConfig feasible, bad_linearize;
	feasible.insert(key, value);
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_nle nle(new NonlinearEquality<VectorConfig>(key, feasible, 2, *vector_compare));

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
	string key = "x";
	Vector value = Vector_(2, 1.0, 2.0);
	Vector wrong = Vector_(2, 3.0, 4.0);
	VectorConfig feasible, bad_linearize;
	feasible.insert(key, value);
	bad_linearize.insert(key, wrong);

	// create a nonlinear equality constraint
	shared_nle nle(new NonlinearEquality<VectorConfig>(key, feasible, 2, *vector_compare));

	// check error function outputs
	Vector actual = nle->error_vector(feasible);
	CHECK(assert_equal(actual, zero(2)));

	actual = nle->error_vector(bad_linearize);
	CHECK(assert_equal(actual, repeat(2, 1.0/0.0)));
}

/* ************************************************************************* */
TEST ( NonlinearEquality, equals ) {
	string key1 = "x";
	Vector value1 = Vector_(2, 1.0, 2.0);
	Vector value2 = Vector_(2, 3.0, 4.0);
	VectorConfig feasible1, feasible2, feasible3;
	feasible1.insert(key1, value1);
	feasible2.insert(key1, value2);
	feasible3.insert(key1, value1);
	feasible3.insert("y", value2);

	// create some constraints to compare
	shared_nle nle1(new NonlinearEquality<VectorConfig>(key1, feasible1, 2, *vector_compare));
	shared_nle nle2(new NonlinearEquality<VectorConfig>(key1, feasible1, 2, *vector_compare));
	shared_nle nle3(new NonlinearEquality<VectorConfig>(key1, feasible2, 2, *vector_compare));
	shared_nle nle4(new NonlinearEquality<VectorConfig>(key1, feasible3, 2, *vector_compare));

	// verify
	CHECK(nle1->equals(*nle2));  // basic equality = true
	CHECK(nle2->equals(*nle1));  // test symmetry of equals()
	CHECK(!nle1->equals(*nle3)); // test config
	CHECK(nle4->equals(*nle1));  // test the full feasible set isn't getting compared
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
