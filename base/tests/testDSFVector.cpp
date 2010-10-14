/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDSF.cpp
 *
 *   Created on: June 25, 2010
 *       Author: nikai
 *  Description: unit tests for DSF
 */

#include <iostream>
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
using namespace boost::assign;
#include <gtsam/CppUnitLite/TestHarness.h>

#include <gtsam/base/DSFVector.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DSFVectorVector, findSet) {
	DSFVector dsf(3);
	CHECK(dsf.findSet(0) != dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVectorVector, makeUnionInPlace) {
	DSFVector dsf(3);
	dsf.makeUnionInPlace(0,2);
	CHECK(dsf.findSet(0) == dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVector, makeUnion2) {
	DSFVector dsf(3);
	dsf.makeUnionInPlace(2,0);
	CHECK(dsf.findSet(0) == dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVector, makeUnion3) {
	DSFVector dsf(3);
	dsf.makeUnionInPlace(0,1);
	dsf.makeUnionInPlace(1,2);
	CHECK(dsf.findSet(0) == dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVector, sets) {
	DSFVector dsf(2);
	dsf.makeUnionInPlace(0,1);
	map<size_t, set<size_t> > sets = dsf.sets();
	LONGS_EQUAL(1, sets.size());

	set<size_t> expected; expected += 0, 1;
	CHECK(expected == sets[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, sets2) {
	DSFVector dsf(3);
	dsf.makeUnionInPlace(0,1);
	dsf.makeUnionInPlace(1,2);
	map<size_t, set<size_t> > sets = dsf.sets();
	LONGS_EQUAL(1, sets.size());

	set<size_t> expected; expected += 0, 1, 2;
	CHECK(expected == sets[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, sets3) {
	DSFVector dsf(3);
	dsf.makeUnionInPlace(0,1);
	map<size_t, set<size_t> > sets = dsf.sets();
	LONGS_EQUAL(2, sets.size());

	set<size_t> expected; expected += 0, 1;
	CHECK(expected == sets[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, set) {
	DSFVector dsf(3);
	dsf.makeUnionInPlace(0,1);
	set<size_t> set = dsf.set(0);
	LONGS_EQUAL(2, set.size());

	std::set<size_t> expected; expected += 0, 1;
	CHECK(expected == set);
}

/* ************************************************************************* */
TEST(DSFVector, set2) {
	DSFVector dsf(3);
	dsf.makeUnionInPlace(0,1);
	dsf.makeUnionInPlace(1,2);
	set<size_t> set = dsf.set(0);
	LONGS_EQUAL(3, set.size());

	std::set<size_t> expected; expected += 0, 1, 2;
	CHECK(expected == set);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

