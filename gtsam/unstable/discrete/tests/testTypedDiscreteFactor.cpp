/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * @file		testTypedDiscreteFactor.cpp
 * @brief		Typed f1s use discrete keys
 * @author	Duy-Nguyen Ta
 * @date	Mar 5, 2011
 */

#include <gtsam_unstable/discrete/TypedDiscreteFactor.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

using namespace std;
using namespace gtsam;

/* ******************************************************************************** */
// initialize some common test values
DiscreteKey v0("v0"), v1("v1"), v2("v2", 3);
TypedDiscreteFactor::Values values;

void init() {
	values[v0] = 0;
	values[v1] = 0;
	values[v2] = 1;
}

/* ******************************************************************************** */
TEST( TypedDiscreteFactor, constructors)
{
	TypedDiscreteFactor f1(v1 & v2, "0.210 0.333 0.457 0.811 0.000 0.189");
	EXPECT_LONGS_EQUAL(2, f1.size());
	//	f1.print();

	double expectedP001 = 0.333;
	EXPECT_DOUBLES_EQUAL(expectedP001, f1(values), 1e-9);

	vector<double> ys;
	ys += 0.210, 0.333, 0.457, 0.811, 0.000, 0.189;
	TypedDiscreteFactor f2(v1 & v2, ys);

	EXPECT(assert_equal(f1, f2, 1e-9));
	EXPECT_LONGS_EQUAL(2, f1.size());
	EXPECT_DOUBLES_EQUAL(expectedP001, f2(values), 1e-9);
}

/* ************************************************************************* */
int main() {
	init();
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
