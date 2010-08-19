/*
 * @file testKey.cpp
 * @author Alex Cunningham
 */

#include <boost/assign/std/list.hpp> // for operator +=
using namespace boost::assign;

#include <gtsam/CppUnitLite/TestHarness.h>
#include <gtsam/inference/Key.h>
	
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
	typedef TypedSymbol<Pose3, 'x'> SimpleKey;
	typedef TypedLabeledSymbol<Pose3, 'x', int> RobotKey;

	SimpleKey key7(1);
	RobotKey key1(0, 1),
			 key2(0, 1),
			 key3(1, 1),
			 key4(2, 1),
			 key5(0, 2),
			 key6(1, 2),
			 key8(1, 3),
			 key9(key7, 3);


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
	CHECK(assert_equal(key9, key8));
}

/* ************************************************************************* */
TEST ( TypedLabledSymbol, encoding ) {
	typedef TypedLabeledSymbol<Pose3, 'x', char> RobotKey;

	RobotKey key1(37, 'A');

	// Note: calculations done in test due to possible differences between machines
	// take the upper two bytes for the label
	short label = key1.label();

	// find the shift necessary
	size_t shift = (sizeof(size_t)-sizeof(short)) * 8;
	size_t modifier = label;
	modifier = modifier << shift;
	size_t index = key1.index() + modifier;

	// check index encoding
	Symbol act1(key1), exp('x', index);
	CHECK(assert_equal(exp, act1));

	// check casting
	Symbol act2 = (Symbol) key1;
	CHECK(assert_equal(exp, act2));

	// decode
	CHECK(assert_equal(key1, RobotKey(act1)));
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

