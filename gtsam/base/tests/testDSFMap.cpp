/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testDSFMap.cpp
 * @date Oct 26, 2013
 * @author Frank Dellaert
 * @brief unit tests for DSFMap
 */

#include <gtsam/base/DSFMap.h>

#include <boost/assign/std/list.hpp>
#include <boost/assign/std/set.hpp>
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(DSFMap, find) {
  DSFMap<size_t> dsf;
  EXPECT(dsf.find(0)==0);
  EXPECT(dsf.find(2)==2);
  EXPECT(dsf.find(0)==0);
  EXPECT(dsf.find(2)==2);
  EXPECT(dsf.find(0) != dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFMap, merge) {
  DSFMap<size_t> dsf;
  dsf.merge(0,2);
  EXPECT(dsf.find(0) == dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFMap, merge2) {
  DSFMap<size_t> dsf;
  dsf.merge(2,0);
  EXPECT(dsf.find(0) == dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFMap, merge3) {
  DSFMap<size_t> dsf;
  dsf.merge(0,1);
  dsf.merge(1,2);
  EXPECT(dsf.find(0) == dsf.find(2));
}

/* ************************************************************************* */
TEST(DSFMap, mergePairwiseMatches) {

  // Create some "matches"
  typedef pair<size_t,size_t> Match;
  list<Match> matches;
  matches += Match(1,2), Match(2,3), Match(4,5), Match(4,6);

  // Merge matches
  DSFMap<size_t> dsf;
  for(const Match& m: matches)
    dsf.merge(m.first,m.second);

  // Each point is now associated with a set, represented by one of its members
  size_t rep1 = dsf.find(1), rep2 = dsf.find(4);
  EXPECT_LONGS_EQUAL(rep1,dsf.find(2));
  EXPECT_LONGS_EQUAL(rep1,dsf.find(3));
  EXPECT_LONGS_EQUAL(rep2,dsf.find(5));
  EXPECT_LONGS_EQUAL(rep2,dsf.find(6));
}

/* ************************************************************************* */
TEST(DSFMap, mergePairwiseMatches2) {

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
  DSFMap<Measurement> dsf;
  for(const Match& m: matches)
    dsf.merge(m.first,m.second);

  // Check that sets are merged correctly
  EXPECT(dsf.find(m22)==dsf.find(m11));
  EXPECT(dsf.find(m23)==dsf.find(m12));
  EXPECT(dsf.find(m25)==dsf.find(m14));
  EXPECT(dsf.find(m26)==dsf.find(m14));
}

/* ************************************************************************* */
TEST(DSFMap, sets){
  // Create some "matches"
  typedef pair<size_t,size_t> Match;
  list<Match> matches;
  matches += Match(1,2), Match(2,3), Match(4,5), Match(4,6);

  // Merge matches
  DSFMap<size_t> dsf;
  for(const Match& m: matches)
    dsf.merge(m.first,m.second);

  map<size_t, set<size_t> > sets = dsf.sets();
  set<size_t> s1, s2;
  s1 += 1,2,3;
  s2 += 4,5,6;

  /*for(key_pair st: sets){
    cout << "Set " << st.first << " :{";
    for(const size_t s: st.second)
      cout << s << ", ";
    cout << "}" << endl;
  }*/

  EXPECT(s1 == sets[1]);
  EXPECT(s2 == sets[4]);
}

/* ************************************************************************* */
TEST(DSFMap, findIndexPair) {
  DSFMap<IndexPair> dsf;
  EXPECT(dsf.find(IndexPair(1,2))==IndexPair(1,2));
  EXPECT(dsf.find(IndexPair(1,2)) != dsf.find(IndexPair(1,3)));
}
/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */

