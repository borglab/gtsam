/*
 * testEqualityFactor.cpp
 *
 *  Created on: Aug 10, 2009
 *      Author: Alex Cunningham
 */


#include <CppUnitLite/TestHarness.h>
#include "EqualityFactor.h"
#include "smallExample.h"

using namespace gtsam;
using namespace std;

TEST ( EqualityFactor, basic )
{
	// create an initialized factor
	Vector v(2); v(0)=1.2; v(1)=3.4;
	string key = "x0";
	EqualityFactor factor(v, key);

	// get the data back out of it
	CHECK(assert_equal(v, factor.get_value()));
	CHECK(key == factor.get_key());
}

TEST ( EqualityFactor, equals )
{
	Vector v(2); v(0)=1.2; v(1)=3.4;
	string key = "x0";
	EqualityFactor factor1(v, key);
	EqualityFactor factor2(v, key);
	CHECK(factor1.equals(factor2));
}

TEST (EqualityFactor, getDeltaFunction )
{
	Vector v(2); v(0)=1.2; v(1)=3.4;
	string key = "x0";
	EqualityFactor factor(v, key);

	DeltaFunction::shared_ptr actual = factor.getDeltaFunction();

	DeltaFunction::shared_ptr expected(new DeltaFunction(v, key));
	CHECK(assert_equal(*actual, *expected));
}

TEST (EqualityFactor, linearize )
{
	FGConfig c = createConstrainedConfig();
	EqualityFactor init(c["x0"], "x0");
	EqualityFactor::shared_ptr actual = init.linearize();
	EqualityFactor::shared_ptr expected(new EqualityFactor(zero(2), "x0"));
	CHECK(assert_equal(*actual, *expected));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

