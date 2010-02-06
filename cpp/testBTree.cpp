/*
 * testBNode.cpp
 *
 *  Created on: Feb 3, 2010
 *      Author: cbeall3
 */

#include <boost/shared_ptr.hpp>
#include <CppUnitLite/TestHarness.h>
#include "Key.h"
#include "BTree.h"

using namespace std;
using namespace gtsam;

typedef pair<size_t, size_t> Range;
//typedef boost::shared_ptr<Node<Symbol, Range> > Tree;
typedef Node<Symbol,Range>::Tree RangeTree;


/* ************************************************************************* */
TEST( BNode, constructor )
{
	RangeTree tree;
	CHECK(tree==NULL)
	LONGS_EQUAL(0,height(tree))

	// check the height of tree after adding an element
	RangeTree tree1 = add(Symbol('x',1), Range(1,2), tree);
  LONGS_EQUAL(1,height(tree1))

	boost::optional<Range> range1 = find(tree1, Symbol('x',1));
  CHECK(range1 == Range(1,2));

  RangeTree tree2 = add(Symbol('x',5), Range(5,6), tree1);
  RangeTree tree3 = add(Symbol('x',3), Range(3,4), tree2);

  boost::optional<Range> range2 = find(tree3, Symbol('x',5));
  boost::optional<Range> range3 = find(tree3, Symbol('x',3));

	CHECK(range2 == Range(5,6));
	CHECK(range3 == Range(3,4));

	// this causes Bus Error.
	//RangeTree tree4 = add(Symbol('x',2), Range(3,4), tree3);

	//walk(tree4, "root");
}

/* ************************************************************************* */
int main() {
	TestResult tr;
	return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
