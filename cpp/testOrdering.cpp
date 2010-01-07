/**
 * @file testOrdering.cpp
 * @author Alex Cunningham
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <CppUnitLite/TestHarness.h>
#include "Ordering.h"

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
// x1 -> x2
//		-> x3 -> x4
//    -> x5
TEST ( Ordering, constructor ) {
	map<string, string> p_map;
	p_map.insert(make_pair("x1", "x1"));
	p_map.insert(make_pair("x2", "x1"));
	p_map.insert(make_pair("x3", "x1"));
	p_map.insert(make_pair("x4", "x3"));
	p_map.insert(make_pair("x5", "x1"));

	Ordering expected;
	expected += "x4", "x5", "x3", "x2", "x1";

	Ordering actual(p_map);
	CHECK(assert_equal(expected, actual));
}


/* ************************************************************************* */
TEST ( Ordering, subtract ) {
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
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
