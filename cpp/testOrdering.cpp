/**
 * @file testOrdering.cpp
 * @author Alex Cunningham
 */

#include <boost/assign/std/list.hpp> // for operator +=
#include <CppUnitLite/TestHarness.h>
#include "Ordering-inl.h"
#include "Pose2Graph.h"


using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
// x1 -> x2
//		-> x3 -> x4
//    -> x5
TEST ( Ordering, predecessorMap2Keys ) {
	typedef Symbol<Pose2,'x'> Key;
	PredecessorMap<Key> p_map;
	p_map.insert(1,1);
	p_map.insert(2,1);
	p_map.insert(3,1);
	p_map.insert(4,3);
	p_map.insert(5,1);

	list<Key> expected;
	expected += 4,5,3,2,1;//Key(4), Key(5), Key(3), Key(2), Key(1);

	list<Key> actual = predecessorMap2Keys<Key>(p_map);
	LONGS_EQUAL(expected.size(), actual.size());

	list<Key>::const_iterator it1 = expected.begin();
	list<Key>::const_iterator it2 = actual.begin();
	for(; it1!=expected.end(); it1++, it2++)
		CHECK(*it1 == *it2)
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
