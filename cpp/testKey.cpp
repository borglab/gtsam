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

/* ************************************************************************* *
TEST ( TypedLabledSymbol, encoding ) {
	typedef TypedLabeledSymbol<Pose3, 'x', char> RobotKey;

	cout << "short : " << sizeof(short) << "  size_t: " << sizeof(size_t) << endl;
	cout << "unsigned int : " << sizeof(unsigned int) << endl;

	RobotKey key1(37, 'A');

	size_t index = key1.index();
	size_t modifier = 65;
	modifier = modifier << sizeof(size_t) * 4;
	index += modifier;
	cout << "index: " << index << "  modifier: " << modifier << endl;


//	short encoded = 65;
//	size_t modifier = encoded << 32;
//	size_t index = 37 + encoded;
//	Symbol act(key1), exp('x', index);
//	CHECK(assert_equal(exp, act));
}


/* ************************************************************************* *
TEST ( TypedLabledSymbol, symbol_translation ) {
	typedef TypedLabeledSymbol<Pose3, 'x', char> Key;

	Key key1(0, 'A'),
		key2(1, 'A'),
		key3(0, 'B'),
		key4(1, 'B');

	LabeledSymbol act1(key1), act2(key2), act3(key3), act4(key4);
	LabeledSymbol exp1('x', 0, 'A'),
				  exp2('x', 1, 'A'),
				  exp3('x', 0, 'B'),
				  exp4('x', 1, 'B');
	CHECK(assert_equal(exp1, act1));
	CHECK(assert_equal(exp2, act2));
	CHECK(assert_equal(exp3, act3));
	CHECK(assert_equal(exp4, act4));

}

/* ************************************************************************* *
TEST ( TypedLabledSymbol, symbol_comparison ) {
	typedef TypedLabeledSymbol<Pose3, 'x', char> Key1;
	typedef TypedSymbol<Pose3, 'x'> Key2;

	Key1 key1(0, 'A'),
		 key2(1, 'A'),
		 key3(0, 'B'),
		 key4(1, 'B');
	Key2 key5(0), key6(1);

	LabeledSymbol act1(key1), act2(key2), act3(key3), act4(key4);

	CHECK(act1 != act2);
	CHECK(act1 != act3);
	CHECK(act1 != act4);
	CHECK(act1 == act1);
	CHECK(act1 < act2);
	CHECK(act1 < act3);
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

