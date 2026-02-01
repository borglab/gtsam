/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBTree.cpp
 * @date Feb 3, 2010
 * @author Chris Beall
 * @author Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam_unstable/base/BTree.h>

#include <list>

using namespace std;
using namespace gtsam;

typedef pair<size_t, size_t> Range;
typedef BTree<string, Range> RangeTree;
typedef BTree<string, int> IntTree;

static std::stringstream ss;
static string x1("x1"), x2("x2"), x3("x3"), x4("x4"), x5("x5");
typedef pair<string, int> KeyInt;
KeyInt p1(x1, 1), p2(x2, 2), p3(x3, 3), p4(x4, 4), p5(x5, 5);

/* ************************************************************************* */
int f(const string& key, const Range& range) {
  return range.first;
}

void g(const string& key, int i) {
  ss << (string) key;
}

int add(const string& k, int v, int a) {
  return v + a;
}

/* ************************************************************************* */
TEST( BTree, add )
{
  RangeTree tree;
  CHECK(tree.empty())
  LONGS_EQUAL(0,tree.height())

  // check the height of tree after adding an element
  RangeTree tree1 = tree.add(x1, Range(1, 1));
  LONGS_EQUAL(1,tree1.height())
  LONGS_EQUAL(1,tree1.size())
  CHECK(tree1.find(x1) == Range(1,1))

  RangeTree tree2 = tree1.add(x5, Range(5, 2));
  RangeTree tree3 = tree2.add(x3, Range(3, 3));
  LONGS_EQUAL(3,tree3.size())
  CHECK(tree3.find(x5) == Range(5,2))
  CHECK(tree3.find(x3) == Range(3,3))

  RangeTree tree4 = tree3.add(x2, Range(2, 4));
  RangeTree tree5 = tree4.add(x4, Range(4, 5));
  LONGS_EQUAL(5,tree5.size())
  CHECK(tree5.find(x4) == Range(4,5))

  // Test functional nature: tree5 and tree6 have different values for x4
  RangeTree tree6 = tree5.add(x4, Range(6, 6));
  CHECK(tree5.find(x4) == Range(4,5))
  CHECK(tree6.find(x4) == Range(6,6))

  // test assignment
  RangeTree c5 = tree5;
  LONGS_EQUAL(5,c5.size())
  CHECK(c5.find(x4) == Range(4,5))

  // test map
  // After (map f tree5) tree contains (x1,1), (x2,2), etc...
  IntTree mapped = tree5.map<int> (f);
  LONGS_EQUAL(2,mapped.find(x2));
  LONGS_EQUAL(4,mapped.find(x4));
}

/* ************************************************************************* */
TEST( BTree, equality )
{
  IntTree tree1 = IntTree().add(p1).add(p2).add(p3).add(p4).add(p5);
  CHECK(tree1==tree1)
  CHECK(tree1.same(tree1))

  IntTree tree2 = IntTree().add(p1).add(p2).add(p3).add(p4).add(p5);
  CHECK(tree2==tree1)
  CHECK(!tree2.same(tree1))

  IntTree tree3 = IntTree().add(p1).add(p2).add(p3).add(p4);
  CHECK(tree3!=tree1)
  CHECK(tree3!=tree2)
  CHECK(!tree3.same(tree1))
  CHECK(!tree3.same(tree2))

  IntTree tree4 = tree3.add(p5);
  CHECK(tree4==tree1)
  CHECK(!tree4.same(tree1))

  IntTree tree5 = tree1;
  CHECK(tree5==tree1)
  CHECK(tree5==tree2)
  CHECK(tree5.same(tree1))
  CHECK(!tree5.same(tree2))
}

/* ************************************************************************* */
TEST( BTree, iterating )
{
  IntTree tree = IntTree().add(p1).add(p2).add(p3).add(p4).add(p5);

  // test iter
  tree.iter(g);
  CHECK(ss.str() == string("x1x2x3x4x5"));

  // test fold
  LONGS_EQUAL(25,tree.fold<int>(add,10))

  // test iterator
  BTree<string, int>::const_iterator it = tree.begin(), it2 = tree.begin();
  CHECK(it==it2)
  CHECK(*it == p1)
  CHECK(it->first == x1)
  CHECK(it->second == 1)
  CHECK(*(++it) == p2)
  CHECK(it!=it2)
  CHECK(it==(++it2))
  CHECK(*(++it) == p3)
  CHECK(*(it++) == p3)
  // post-increment, not so efficient
  CHECK(*it == p4)
  CHECK(*(++it) == p5)
  CHECK((++it)==tree.end())

  // acid iterator test
  int sum = 0;
  for (const KeyInt& p : tree) sum += p.second;
  LONGS_EQUAL(15,sum)

  // STL iterator test
  auto expected = std::list<KeyInt> {p1, p2, p3, p4, p5};
  std::list<KeyInt> actual;
  copy (tree.begin(),tree.end(),back_inserter(actual));
  CHECK(actual==expected)
}

/* ************************************************************************* */
TEST( BTree, remove )
{
  IntTree tree5 = IntTree().add(p1).add(p2).add(p3).add(p4).add(p5);
  LONGS_EQUAL(5,tree5.size())
  CHECK(tree5.mem(x3))
  IntTree tree4 = tree5.remove(x3);
  LONGS_EQUAL(4,tree4.size())
  CHECK(!tree4.mem(x3))
}

/* ************************************************************************* */
TEST( BTree, stress )
{
  RangeTree tree;
  list<RangeTree::value_type> expected;
  int N = 128;
  for (int i = 1; i <= N; i++) {
    string key('a', i);
    Range value(i - 1, i);
    tree = tree.add(key, value);
    LONGS_EQUAL(i,tree.size())
    CHECK(tree.find(key) == value)
    expected.emplace_back(key, value);
  }

  // Check height is log(N)
  LONGS_EQUAL(8,tree.height())

  // stress test iterator
  list<RangeTree::value_type> actual;
  copy(tree.begin(), tree.end(), back_inserter(actual));
  CHECK(actual==expected)

  // deconstruct the tree
  for (int i = N; i >= N; i--) {
    string key('a', i);
    tree = tree.remove(key);
    LONGS_EQUAL(i-1,tree.size())
    CHECK(!tree.mem(key))
  }
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
