/*
 * testDSF.cpp
 *
 *   Created on: Mar 26, 2010
 *       Author: nikai
 *  Description: unit tests for DSF
 */

#include <iostream>
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
using namespace boost::assign;
#include <CppUnitLite/TestHarness.h>

#include "DSF.h"

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DSF, makeSet) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	LONGS_EQUAL(1, dsf.size());
}

/* ************************************************************************* */
TEST(DSF, findSet) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	CHECK(dsf.findSet(5) != dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makeUnion) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,7);
	CHECK(dsf.findSet(5) == dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makeUnion2) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(7,5);
	CHECK(dsf.findSet(5) == dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makeUnion3) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,6);
	dsf = dsf.makeUnion(6,7);
	CHECK(dsf.findSet(5) == dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makePair) {
	DSFInt dsf;
	dsf = dsf.makePair(0, 1);
	dsf = dsf.makePair(1, 2);
	dsf = dsf.makePair(3, 2);
	CHECK(dsf.findSet(0) == dsf.findSet(3));
}

/* ************************************************************************* */
TEST(DSF, makeList) {
	DSFInt dsf;
	list<int> keys; keys += 5, 6, 7;
	dsf = dsf.makeList(keys);
	CHECK(dsf.findSet(5) == dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, numSets) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,6);
	LONGS_EQUAL(2, dsf.numSets());
}

/* ************************************************************************* */
TEST(DSF, sets) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeUnion(5,6);
	map<int, set<int> > sets = dsf.sets();
	LONGS_EQUAL(1, sets.size());

	set<int> expected; expected += 5, 6;
	CHECK(expected == sets[dsf.findSet(5)]);
}

/* ************************************************************************* */
TEST(DSF, sets2) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,6);
	dsf = dsf.makeUnion(6,7);
	map<int, set<int> > sets = dsf.sets();
	LONGS_EQUAL(1, sets.size());

	set<int> expected; expected += 5, 6, 7;
	CHECK(expected == sets[dsf.findSet(5)]);
}

/* ************************************************************************* */
TEST(DSF, sets3) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,6);
	map<int, set<int> > sets = dsf.sets();
	LONGS_EQUAL(2, sets.size());

	set<int> expected; expected += 5, 6;
	CHECK(expected == sets[dsf.findSet(5)]);
}

/* ************************************************************************* */
TEST(DSF, partition) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeUnion(5,6);

	list<int> keys; keys += 5;
	map<int, set<int> > partitions = dsf.partition(keys);
	LONGS_EQUAL(1, partitions.size());

	set<int> expected; expected += 5;
	CHECK(expected == partitions[dsf.findSet(5)]);
}

/* ************************************************************************* */
TEST(DSF, partition2) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,6);

	list<int> keys; keys += 7;
	map<int, set<int> > partitions = dsf.partition(keys);
	LONGS_EQUAL(1, partitions.size());

	set<int> expected; expected += 7;
	CHECK(expected == partitions[dsf.findSet(7)]);
}

/* ************************************************************************* */
TEST(DSF, partition3) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,6);

	list<int> keys; keys += 5, 7;
	map<int, set<int> > partitions = dsf.partition(keys);
	LONGS_EQUAL(2, partitions.size());

	set<int> expected; expected += 5;
	CHECK(expected == partitions[dsf.findSet(5)]);
}

/* ************************************************************************* */
int func(const int& a) { return a + 10; }
TEST(DSF, map) {
	DSFInt dsf;
	dsf = dsf.makeSet(5);
	dsf = dsf.makeSet(6);
	dsf = dsf.makeSet(7);
	dsf = dsf.makeUnion(5,6);

	DSFInt actual = dsf.map(&func);
	DSFInt expected;
	dsf = dsf.makeSet(15);
	dsf = dsf.makeSet(16);
	dsf = dsf.makeSet(17);
	dsf = dsf.makeUnion(15,16);
	CHECK(actual == expected);
}

/* ************************************************************************* */
TEST(DSF, flatten) {
	DSFInt dsf;
	dsf = dsf.makePair(1, 2);
	dsf = dsf.makePair(2, 3);
	dsf = dsf.makePair(5, 6);
	dsf = dsf.makePair(6, 7);
	dsf = dsf.makeUnion(2, 6);

	DSFInt actual = dsf.flatten();
	DSFInt expected;
	expected = expected.makePair(1, 2);
	expected = expected.makePair(1, 3);
	expected = expected.makePair(1, 5);
	expected = expected.makePair(1, 6);
	expected = expected.makePair(1, 7);
	CHECK(actual == expected);
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

