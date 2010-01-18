/*
 * @file testKey.cpp
 * @author Alex Cunningham
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include "Key.h"
	
using namespace std;
using namespace gtsam;

class Pose3;

/* ************************************************************************* */
TEST ( TypedSymbol, basic_operations ) {
	typedef TypedSymbol<Pose3, 'x'> Key;

	Key key1(0),
		key2(0),
		key3(1),
		key4(2);

	CHECK(key1.index()==0);
	CHECK(key1 == key2);
	CHECK(assert_equal(key1, key2));
	CHECK(!(key1 == key3));
	CHECK(key1 < key3);
	CHECK(key3 < key4);
}

/* ************************************************************************* */
TEST ( TypedLabledSymbol, basic_operations ) {
	typedef TypedLabeledSymbol<Pose3, 'x', int> RobotKey;

	RobotKey key1(0, 1),
			 key2(0, 1),
			 key3(1, 1),
			 key4(2, 1),
			 key5(0, 2),
			 key6(1, 2);

	CHECK(key1.label()==1);
	CHECK(key1.index()==0);
	CHECK(key1 == key2);
	CHECK(assert_equal(key1, key2));
	CHECK(!(key1 == key3));
	CHECK(key1 < key3);
	CHECK(key3 < key4);
	CHECK(!(key1 == key5));
	CHECK(key1 < key5);
	CHECK(key5 < key6);
}

/* ************************************************************************* */
TEST ( Key, keys2symbols )
{
	typedef TypedSymbol<int, 'x'> Key;
	list<Symbol> expected;
	expected += Key(1), Key(2), Key(3);

	list<TypedSymbol<int, 'x'> > typeds;
	typeds += 1, 2, 3;
	CHECK(expected == keys2symbols(typeds));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
