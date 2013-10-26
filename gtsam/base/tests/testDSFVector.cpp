/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testDSF.cpp
 * @date June 25, 2010
 * @author nikai
 * @brief unit tests for DSF
 */

#include <gtsam/base/DSFVector.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DSFVectorVector, findSet) {
  DSFVector dsf(3);
  EXPECT(dsf.findSet(0) != dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVectorVector, makeUnionInPlace) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,2);
  EXPECT(dsf.findSet(0) == dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVectorVector, makeUnionInPlace2) {
  boost::shared_ptr<DSFVector::V> v = boost::make_shared<DSFVector::V>(5);
  std::vector<size_t> keys; keys += 1, 3;
  DSFVector dsf(v, keys);
  dsf.makeUnionInPlace(1,3);
  EXPECT(dsf.findSet(1) == dsf.findSet(3));
}

/* ************************************************************************* */
TEST(DSFVector, makeUnion2) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(2,0);
  EXPECT(dsf.findSet(0) == dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVector, makeUnion3) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  dsf.makeUnionInPlace(1,2);
  EXPECT(dsf.findSet(0) == dsf.findSet(2));
}

/* ************************************************************************* */
TEST(DSFVector, sets) {
  DSFVector dsf(2);
  dsf.makeUnionInPlace(0,1);
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(1, sets.size());

  set<size_t> expected; expected += 0, 1;
  EXPECT(expected == sets[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, arrays) {
  DSFVector dsf(2);
  dsf.makeUnionInPlace(0,1);
  map<size_t, vector<size_t> > arrays = dsf.arrays();
  LONGS_EQUAL(1, arrays.size());

  vector<size_t> expected; expected += 0, 1;
  EXPECT(expected == arrays[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, sets2) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  dsf.makeUnionInPlace(1,2);
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(1, sets.size());

  set<size_t> expected; expected += 0, 1, 2;
  EXPECT(expected == sets[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, arrays2) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  dsf.makeUnionInPlace(1,2);
  map<size_t, vector<size_t> > arrays = dsf.arrays();
  LONGS_EQUAL(1, arrays.size());

  vector<size_t> expected; expected += 0, 1, 2;
  EXPECT(expected == arrays[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, sets3) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(2, sets.size());

  set<size_t> expected; expected += 0, 1;
  EXPECT(expected == sets[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, arrays3) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  map<size_t, vector<size_t> > arrays = dsf.arrays();
  LONGS_EQUAL(2, arrays.size());

  vector<size_t> expected; expected += 0, 1;
  EXPECT(expected == arrays[dsf.findSet(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, set) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  set<size_t> set = dsf.set(0);
  LONGS_EQUAL(2, set.size());

  std::set<size_t> expected; expected += 0, 1;
  EXPECT(expected == set);
}

/* ************************************************************************* */
TEST(DSFVector, set2) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  dsf.makeUnionInPlace(1,2);
  set<size_t> set = dsf.set(0);
  LONGS_EQUAL(3, set.size());

  std::set<size_t> expected; expected += 0, 1, 2;
  EXPECT(expected == set);
}

/* ************************************************************************* */
TEST(DSFVector, isSingleton) {
  DSFVector dsf(3);
  dsf.makeUnionInPlace(0,1);
  EXPECT(!dsf.isSingleton(0));
  EXPECT(!dsf.isSingleton(1));
  EXPECT( dsf.isSingleton(2));
}

/* ************************************************************************* */
TEST(DSFVector, mergePairwiseMatches) {

  // Create some measurements
  vector<size_t> keys;
  keys += 1,2,3,4,5,6;

  // Create some "matches"
  typedef pair<size_t,size_t> Match;
  vector<Match> matches;
  matches += Match(1,2), Match(2,3), Match(4,5), Match(4,6);

  // Merge matches
  DSFVector dsf(keys);
  BOOST_FOREACH(const Match& m, matches)
    dsf.makeUnionInPlace(m.first,m.second);

  // Each point is now associated with a set, represented by one of its members
  size_t rep1 = 1, rep2 = 4;
  EXPECT_LONGS_EQUAL(rep1,dsf.findSet(1));
  EXPECT_LONGS_EQUAL(rep1,dsf.findSet(2));
  EXPECT_LONGS_EQUAL(rep1,dsf.findSet(3));
  EXPECT_LONGS_EQUAL(rep2,dsf.findSet(4));
  EXPECT_LONGS_EQUAL(rep2,dsf.findSet(5));
  EXPECT_LONGS_EQUAL(rep2,dsf.findSet(6));

  // Check that we have two connected components, 1,2,3 and 4,5,6
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(2, sets.size());
  set<size_t> expected1; expected1 += 1,2,3;
  set<size_t> actual1 = sets[rep1];
  EXPECT(expected1 == actual1);
  set<size_t> expected2; expected2 += 4,5,6;
  set<size_t> actual2 = sets[rep2];
  EXPECT(expected2 == actual2);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

