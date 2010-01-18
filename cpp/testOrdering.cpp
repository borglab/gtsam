/**
 * @file testOrdering.cpp
 * @author Alex Cunningham
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

// Magically turn strings into Symbols
#define GTSAM_MAGIC_KEY

#include "Ordering.h"
#include "pose2SLAM.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST ( Ordering, subtract )
{
	Ordering init, delta;
	init += "a", "b", "c", "d", "e";
	CHECK(assert_equal(init.subtract(delta), init));

	delta += "b";
	Ordering expected1;
	expected1 += "a", "c", "d", "e";
	CHECK(assert_equal(init.subtract(delta), expected1));

	delta += "e";
	Ordering expected2;
	expected2 += "a", "c", "d";
	CHECK(assert_equal(init.subtract(delta), expected2));
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
