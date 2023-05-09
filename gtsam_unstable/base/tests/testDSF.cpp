/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testDSF.cpp
 * @date Mar 26, 2010
 * @author nikai
 * @brief unit tests for DSF
 */

#include <gtsam_unstable/base/DSF.h>

#include <CppUnitLite/TestHarness.h>

#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
using namespace boost::assign;

#include <iostream>

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
  EXPECT(dsf.findSet(5) != dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makeUnion) {
  DSFInt dsf;
  dsf = dsf.makeSet(5);
  dsf = dsf.makeSet(6);
  dsf = dsf.makeSet(7);
  dsf = dsf.makeUnion(5,7);
  EXPECT(dsf.findSet(5) == dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makeUnion2) {
  DSFInt dsf;
  dsf = dsf.makeSet(5);
  dsf = dsf.makeSet(6);
  dsf = dsf.makeSet(7);
  dsf = dsf.makeUnion(7,5);
  EXPECT(dsf.findSet(5) == dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makeUnion3) {
  DSFInt dsf;
  dsf = dsf.makeSet(5);
  dsf = dsf.makeSet(6);
  dsf = dsf.makeSet(7);
  dsf = dsf.makeUnion(5,6);
  dsf = dsf.makeUnion(6,7);
  EXPECT(dsf.findSet(5) == dsf.findSet(7));
}

/* ************************************************************************* */
TEST(DSF, makePair) {
  DSFInt dsf;
  dsf = dsf.makePair(0, 1);
  dsf = dsf.makePair(1, 2);
  dsf = dsf.makePair(3, 2);
  EXPECT(dsf.findSet(0) == dsf.findSet(3));
}

/* ************************************************************************* */
TEST(DSF, makeList) {
  DSFInt dsf;
  list<int> keys; keys += 5, 6, 7;
  dsf = dsf.makeList(keys);
  EXPECT(dsf.findSet(5) == dsf.findSet(7));
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
  EXPECT(expected == sets[dsf.findSet(5)]);
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
  EXPECT(expected == sets[dsf.findSet(5)]);
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
  EXPECT(expected == sets[dsf.findSet(5)]);
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
  EXPECT(expected == partitions[dsf.findSet(5)]);
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
  EXPECT(expected == partitions[dsf.findSet(7)]);
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
  EXPECT(expected == partitions[dsf.findSet(5)]);
}

/* ************************************************************************* */
TEST(DSF, set) {
  DSFInt dsf;
  dsf = dsf.makeSet(5);
  dsf = dsf.makeSet(6);
  dsf = dsf.makeSet(7);
  dsf = dsf.makeUnion(5,6);
  set<int> set = dsf.set(5);
  LONGS_EQUAL(2, set.size());

  std::set<int> expected; expected += 5, 6;
  EXPECT(expected == set);
}

/* ************************************************************************* */
TEST(DSF, set2) {
  DSFInt dsf;
  dsf = dsf.makeSet(5);
  dsf = dsf.makeSet(6);
  dsf = dsf.makeSet(7);
  dsf = dsf.makeUnion(5,6);
  dsf = dsf.makeUnion(6,7);
  set<int> set = dsf.set(5);
  LONGS_EQUAL(3, set.size());

  std::set<int> expected; expected += 5, 6, 7;
  EXPECT(expected == set);
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
  expected = expected.makeSet(15);
  expected = expected.makeSet(16);
  expected = expected.makeSet(17);
  expected = expected.makeUnion(15,16);
  EXPECT(actual == expected);
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
  EXPECT(actual == expected);
}

/* ************************************************************************* */
TEST(DSF, flatten2) {
  static string x1("x1"), x2("x2"), x3("x3"), x4("x4");
  list<string> keys; keys += x1,x2,x3,x4;
  DSF<string> dsf(keys);
  dsf = dsf.makeUnion(x1,x2);
  dsf = dsf.makeUnion(x3,x4);
  dsf = dsf.makeUnion(x1,x3);

  EXPECT(dsf != dsf.flatten());

  DSF<string> expected2;
  expected2 = expected2.makePair(x1, x2);
  expected2 = expected2.makePair(x1, x3);
  expected2 = expected2.makePair(x1, x4);
  EXPECT(expected2 == dsf.flatten());
}

/* ************************************************************************* */
TEST(DSF, mergePairwiseMatches) {

  // Create some measurements with image index and feature index
  typedef pair<size_t,size_t> Measurement;
  Measurement m11(1,1),m12(1,2),m14(1,4); // in image 1
  Measurement m22(2,2),m23(2,3),m25(2,5),m26(2,6); // in image 2

  // Add them all
  list<Measurement> measurements;
  measurements += m11,m12,m14, m22,m23,m25,m26;

  // Create some "matches"
  typedef pair<Measurement,Measurement> Match;
  list<Match> matches;
  matches += Match(m11,m22), Match(m12,m23), Match(m14,m25), Match(m14,m26);

  // Merge matches
  DSF<Measurement> dsf(measurements);
  for(const Match& m: matches)
    dsf.makeUnionInPlace(m.first,m.second);

  // Check that sets are merged correctly
  EXPECT(dsf.findSet(m11)==m11);
  EXPECT(dsf.findSet(m12)==m12);
  EXPECT(dsf.findSet(m14)==m14);
  EXPECT(dsf.findSet(m22)==m11);
  EXPECT(dsf.findSet(m23)==m12);
  EXPECT(dsf.findSet(m25)==m14);
  EXPECT(dsf.findSet(m26)==m14);

  // Check that we have three connected components
  EXPECT_LONGS_EQUAL(3, dsf.numSets());

  set<Measurement> expected1; expected1 += m11,m22;
  set<Measurement> actual1 = dsf.set(m11);
  EXPECT(expected1 == actual1);

  set<Measurement> expected2; expected2 += m12,m23;
  set<Measurement> actual2 = dsf.set(m12);
  EXPECT(expected2 == actual2);

  set<Measurement> expected3; expected3 += m14,m25,m26;
  set<Measurement> actual3 = dsf.set(m14);
  EXPECT(expected3 == actual3);
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

