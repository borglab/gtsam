/*
 * testDeltaFunction.cpp
 *
 *  Created on: Aug 11, 2009
 *      Author: alexgc
 */


#include <CppUnitLite/TestHarness.h>
#include "DeltaFunction.h"

using namespace gtsam;

TEST (DeltaFunction, basic)
{
	Vector v(2); v(0)=1.0; v(1)=2.0;

	DeltaFunction delta(v, "x");

	CHECK(assert_equal(v, delta.get_value()));
}


/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

