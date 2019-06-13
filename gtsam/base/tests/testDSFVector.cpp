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

#include <boost/make_shared.hpp>
#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign;

#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DSFBase, find) {
  DSFBase dsf(3);
  EXPECT(dsf.find(0) != dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFBase, merge) {
  DSFBase dsf(3);
  dsf.merge(0,2);
  EXPECT(dsf.find(0) == dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFBase, makeUnion2) {
  DSFBase dsf(3);
  dsf.merge(2,0);
  EXPECT(dsf.find(0) == dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFBase, makeUnion3) {
  DSFBase dsf(3);
  dsf.merge(0,1);
  dsf.merge(1,2);
  EXPECT(dsf.find(0) == dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFBase, mergePairwiseMatches) {

  // Create some "matches"
  typedef pair<size_t,size_t> Match;
  vector<Match> matches;
  matches += Match(1,2), Match(2,3), Match(4,5), Match(4,6);

  // Merge matches
  DSFBase dsf(7); // We allow for keys 0..6
  for(const Match& m: matches)
    dsf.merge(m.first,m.second);

  // Each point is now associated with a set, represented by one of its members
  size_t rep1 = 1, rep2 = 4;
  EXPECT_LONGS_EQUAL(rep1,dsf.find(1));
  EXPECT_LONGS_EQUAL(rep1,dsf.find(2));
  EXPECT_LONGS_EQUAL(rep1,dsf.find(3));
  EXPECT_LONGS_EQUAL(rep2,dsf.find(4));
  EXPECT_LONGS_EQUAL(rep2,dsf.find(5));
  EXPECT_LONGS_EQUAL(rep2,dsf.find(6));
}

/* ************************************************************************* */
TEST(DSFVector, merge2) {
  boost::shared_ptr<DSFBase::V> v = boost::make_shared<DSFBase::V>(5);
  std::vector<size_t> keys; keys += 1, 3;
  DSFVector dsf(v, keys);
  dsf.merge(1,3);
  EXPECT(dsf.find(1) == dsf.find(3));
}

/* ************************************************************************* */
TEST(DSFVector, sets) {
  DSFVector dsf(2);
  dsf.merge(0,1);
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(1, sets.size());

  set<size_t> expected; expected += 0, 1;
  EXPECT(expected == sets[dsf.find(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, arrays) {
  DSFVector dsf(2);
  dsf.merge(0,1);
  map<size_t, vector<size_t> > arrays = dsf.arrays();
  LONGS_EQUAL(1, arrays.size());

  vector<size_t> expected; expected += 0, 1;
  EXPECT(expected == arrays[dsf.find(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, sets2) {
  DSFVector dsf(3);
  dsf.merge(0,1);
  dsf.merge(1,2);
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(1, sets.size());

  set<size_t> expected; expected += 0, 1, 2;
  EXPECT(expected == sets[dsf.find(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, arrays2) {
  DSFVector dsf(3);
  dsf.merge(0,1);
  dsf.merge(1,2);
  map<size_t, vector<size_t> > arrays = dsf.arrays();
  LONGS_EQUAL(1, arrays.size());

  vector<size_t> expected; expected += 0, 1, 2;
  EXPECT(expected == arrays[dsf.find(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, sets3) {
  DSFVector dsf(3);
  dsf.merge(0,1);
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(2, sets.size());

  set<size_t> expected; expected += 0, 1;
  EXPECT(expected == sets[dsf.find(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, arrays3) {
  DSFVector dsf(3);
  dsf.merge(0,1);
  map<size_t, vector<size_t> > arrays = dsf.arrays();
  LONGS_EQUAL(2, arrays.size());

  vector<size_t> expected; expected += 0, 1;
  EXPECT(expected == arrays[dsf.find(0)]);
}

/* ************************************************************************* */
TEST(DSFVector, set) {
  DSFVector dsf(3);
  dsf.merge(0,1);
  set<size_t> set = dsf.set(0);
  LONGS_EQUAL(2, set.size());

  std::set<size_t> expected; expected += 0, 1;
  EXPECT(expected == set);
}

/* ************************************************************************* */
TEST(DSFVector, set2) {
  DSFVector dsf(3);
  dsf.merge(0,1);
  dsf.merge(1,2);
  set<size_t> set = dsf.set(0);
  LONGS_EQUAL(3, set.size());

  std::set<size_t> expected; expected += 0, 1, 2;
  EXPECT(expected == set);
}

/* ************************************************************************* */
TEST(DSFVector, isSingleton) {
  DSFVector dsf(3);
  dsf.merge(0,1);
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
  for(const Match& m: matches)
    dsf.merge(m.first,m.second);

  // Check that we have two connected components, 1,2,3 and 4,5,6
  map<size_t, set<size_t> > sets = dsf.sets();
  LONGS_EQUAL(2, sets.size());
  set<size_t> expected1; expected1 += 1,2,3;
  set<size_t> actual1 = sets[dsf.find(2)];
  EXPECT(expected1 == actual1);
  set<size_t> expected2; expected2 += 4,5,6;
  set<size_t> actual2 = sets[dsf.find(5)];
  EXPECT(expected2 == actual2);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

