/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testOrdering
 * @author Alex Cunningham
 * @author Andrew Melim
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/inference/MetisIndex.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

namespace example {
SymbolicFactorGraph symbolicChain() {
  SymbolicFactorGraph symbolicGraph;
  symbolicGraph.push_factor(0, 1);
  symbolicGraph.push_factor(1, 2);
  symbolicGraph.push_factor(2, 3);
  symbolicGraph.push_factor(3, 4);
  symbolicGraph.push_factor(4, 5);
  return symbolicGraph;
}
}
/* ************************************************************************* */
TEST(Ordering, constrained_ordering) {
  // create graph with wanted variable set = 2, 4
  SymbolicFactorGraph symbolicGraph = example::symbolicChain();

  // unconstrained version
  {
  Ordering actual = Ordering::Colamd(symbolicGraph);
  Ordering expected{0, 1, 2, 3, 4, 5};
  EXPECT(assert_equal(expected, actual));
  }

  // constrained version - push one set to the end
  {
  Ordering actual = Ordering::ColamdConstrainedLast(symbolicGraph, {2, 4});
  Ordering expected = Ordering({0, 1, 5, 3, 4, 2});
  EXPECT(assert_equal(expected, actual));
  }

  // constrained version - push one set to the start
  {
    Ordering actual = Ordering::ColamdConstrainedFirst(symbolicGraph, {2, 4});
    Ordering expected = Ordering({2, 4, 0, 1, 3, 5});
    EXPECT(assert_equal(expected, actual));
  }

  // Make sure giving empty constraints does not break the code
  {
    Ordering actual = Ordering::ColamdConstrainedLast(symbolicGraph, {});
    Ordering expected = Ordering({0, 1, 2, 3, 4, 5});
    EXPECT(assert_equal(expected, actual));
  }
  {
    Ordering actual = Ordering::ColamdConstrainedFirst(symbolicGraph, {});
    Ordering expected = Ordering({0, 1, 2, 3, 4, 5});
    EXPECT(assert_equal(expected, actual));
  }

  // Make sure giving empty graph does not break the code
  SymbolicFactorGraph emptyGraph;
  Ordering empty;
  {
    Ordering actual = Ordering::ColamdConstrainedLast(emptyGraph, {2, 4});
    EXPECT(assert_equal(empty, actual));
  }
  {
    Ordering actual = Ordering::ColamdConstrainedFirst(emptyGraph, {2, 4});
    EXPECT(assert_equal(empty, actual));
  }
}

/* ************************************************************************* */
TEST(Ordering, grouped_constrained_ordering) {

  // create graph with constrained groups:
  // 1: 2, 4
  // 2: 5
  SymbolicFactorGraph symbolicGraph = example::symbolicChain();

  // constrained version - push one set to the end
  FastMap<Key, int> constraints;
  constraints[2] = 1;
  constraints[4] = 1;
  constraints[5] = 2;

  Ordering actual = Ordering::ColamdConstrained(symbolicGraph, constraints);
  Ordering expected{0, 1, 3, 2, 4, 5};
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Ordering, csr_format) {
  // Example in METIS manual
  SymbolicFactorGraph symbolicGraph;
  symbolicGraph.push_factor(0, 1);
  symbolicGraph.push_factor(1, 2);
  symbolicGraph.push_factor(2, 3);
  symbolicGraph.push_factor(3, 4);
  symbolicGraph.push_factor(5, 6);
  symbolicGraph.push_factor(6, 7);
  symbolicGraph.push_factor(7, 8);
  symbolicGraph.push_factor(8, 9);
  symbolicGraph.push_factor(10, 11);
  symbolicGraph.push_factor(11, 12);
  symbolicGraph.push_factor(12, 13);
  symbolicGraph.push_factor(13, 14);

  symbolicGraph.push_factor(0, 5);
  symbolicGraph.push_factor(5, 10);
  symbolicGraph.push_factor(1, 6);
  symbolicGraph.push_factor(6, 11);
  symbolicGraph.push_factor(2, 7);
  symbolicGraph.push_factor(7, 12);
  symbolicGraph.push_factor(3, 8);
  symbolicGraph.push_factor(8, 13);
  symbolicGraph.push_factor(4, 9);
  symbolicGraph.push_factor(9, 14);

  MetisIndex mi(symbolicGraph);

  const vector<int> xadjExpected{0,  2,  5,  8,  11, 13, 16, 20,
                                 24, 28, 31, 33, 36, 39, 42, 44},
      adjExpected{1,  5, 0,  2, 6,  1,  3, 7,  2,  4, 8,  3,  9,  0, 6,
                  10, 1, 5,  7, 11, 2,  6, 8,  12, 3, 7,  9,  13, 4, 8,
                  14, 5, 11, 6, 10, 12, 7, 11, 13, 8, 12, 14, 9,  13};

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == mi.adj());
}

/* ************************************************************************* */
TEST(Ordering, csr_format_2) {
  SymbolicFactorGraph symbolicGraph;

  symbolicGraph.push_factor(0);
  symbolicGraph.push_factor(0, 1);
  symbolicGraph.push_factor(1, 2);
  symbolicGraph.push_factor(2, 3);
  symbolicGraph.push_factor(3, 4);
  symbolicGraph.push_factor(4, 1);

  MetisIndex mi(symbolicGraph);

  const std::vector<int> xadjExpected{0, 1, 4, 6, 8, 10},
      adjExpected{1, 0, 2, 4, 1, 3, 2, 4, 1, 3};

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == mi.adj());
}

/* ************************************************************************* */
TEST(Ordering, csr_format_3) {
  SymbolicFactorGraph symbolicGraph;

  symbolicGraph.push_factor(100);
  symbolicGraph.push_factor(100, 101);
  symbolicGraph.push_factor(101, 102);
  symbolicGraph.push_factor(102, 103);
  symbolicGraph.push_factor(103, 104);
  symbolicGraph.push_factor(104, 101);

  MetisIndex mi(symbolicGraph);

  const std::vector<int> xadjExpected{0, 1, 4, 6, 8, 10},
      adjExpected{1, 0, 2, 4, 1, 3, 2, 4, 1, 3};
  //size_t minKey = mi.minKey();

  vector<int> adjAcutal = mi.adj();

  // Normalize, subtract the smallest key
  //std::transform(adjAcutal.begin(), adjAcutal.end(), adjAcutal.begin(),
  //    std::bind2nd(std::minus<size_t>(), minKey));

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == adjAcutal);
}

/* ************************************************************************* */
TEST(Ordering, AppendVector) {
  using symbol_shorthand::X;
  KeyVector keys{X(0), X(1), X(2)};
  Ordering actual;
  actual += keys;

  Ordering expected{X(0), X(1), X(2)};
  EXPECT(assert_equal(expected, actual));
}

/* ************************************************************************* */
TEST(Ordering, Contains) {
  using symbol_shorthand::X;
  Ordering ordering{X(0), X(1), X(2)};

  EXPECT(ordering.contains(X(1)));
  EXPECT(!ordering.contains(X(4)));
}

/* ************************************************************************* */
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
TEST(Ordering, csr_format_4) {
  SymbolicFactorGraph symbolicGraph;

  symbolicGraph.push_factor(Symbol('x', 1));
  symbolicGraph.push_factor(Symbol('x', 1), Symbol('x', 2));
  symbolicGraph.push_factor(Symbol('x', 2), Symbol('x', 3));
  symbolicGraph.push_factor(Symbol('x', 3), Symbol('x', 4));
  symbolicGraph.push_factor(Symbol('x', 4), Symbol('x', 5));
  symbolicGraph.push_factor(Symbol('x', 5), Symbol('x', 6));

  MetisIndex mi(symbolicGraph);

  const vector<int> xadjExpected{0, 1, 3, 5, 7, 9, 10},
      adjExpected{1, 0, 2, 1, 3, 2, 4, 3, 5, 4};

  vector<int> adjAcutal = mi.adj();
  vector<int> xadjActual = mi.xadj();

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == adjAcutal);

  Ordering metOrder = Ordering::Metis(symbolicGraph);

  // Test different symbol types
  symbolicGraph.push_factor(Symbol('l', 1));
  symbolicGraph.push_factor(Symbol('x', 1), Symbol('l', 1));
  symbolicGraph.push_factor(Symbol('x', 2), Symbol('l', 1));
  symbolicGraph.push_factor(Symbol('x', 3), Symbol('l', 1));
  symbolicGraph.push_factor(Symbol('x', 4), Symbol('l', 1));

  Ordering metOrder2 = Ordering::Metis(symbolicGraph);
}
#endif
/* ************************************************************************* */
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
TEST(Ordering, metis) {

  SymbolicFactorGraph symbolicGraph;

  symbolicGraph.push_factor(0);
  symbolicGraph.push_factor(0, 1);
  symbolicGraph.push_factor(1, 2);

  MetisIndex mi(symbolicGraph);

  const vector<int> xadjExpected{0, 1, 3, 4}, adjExpected{1, 0, 2, 1};

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == mi.adj());

  Ordering metis = Ordering::Metis(symbolicGraph);
}
#endif
/* ************************************************************************* */
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
TEST(Ordering, MetisLoop) {

  // create linear graph
  SymbolicFactorGraph symbolicGraph = example::symbolicChain();

  // add loop closure
  symbolicGraph.push_factor(0, 5);

  // METIS
#if defined(__APPLE__)
  {
    Ordering actual = Ordering::Create(Ordering::METIS, symbolicGraph);
    //  - P( 1 0 3)
    //  | - P( 4 | 0 3)
    //  | | - P( 5 | 0 4)
    //  | - P( 2 | 1 3)
    Ordering expected = Ordering({5, 4, 2, 1, 0, 3});
    EXPECT(assert_equal(expected, actual));
  }
#elif defined(_WIN32)
  {
    Ordering actual = Ordering::Create(Ordering::METIS, symbolicGraph);
    //  - P( 0 5 2)
    //  | - P( 3 | 5 2)
    //  | | - P( 4 | 5 3)
    //  | - P( 1 | 0 2)
    Ordering expected = Ordering({4, 3, 1, 0, 5, 2});
    EXPECT(assert_equal(expected, actual));
  }
#else
  {
    Ordering actual = Ordering::Create(Ordering::METIS, symbolicGraph);
    //  - P( 0 4 1)
    //  | - P( 2 | 4 1)
    //  | | - P( 3 | 4 2)
    //  | - P( 5 | 0 1)
    Ordering expected = Ordering({3, 2, 5, 0, 4, 1});
    EXPECT(assert_equal(expected, actual));
  }
#endif
}
#endif
/* ************************************************************************* */
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
TEST(Ordering, MetisEmptyGraph) {
  SymbolicFactorGraph symbolicGraph;

  Ordering actual = Ordering::Create(Ordering::METIS, symbolicGraph);
  Ordering expected;
  EXPECT(assert_equal(expected, actual));
}
#endif
/* ************************************************************************* */
#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
TEST(Ordering, MetisSingleNode) {
  // create graph with a single node
  SymbolicFactorGraph symbolicGraph;
  symbolicGraph.push_factor(7);

  Ordering actual = Ordering::Create(Ordering::METIS, symbolicGraph);
  Ordering expected = Ordering({7});
  EXPECT(assert_equal(expected, actual));
}
#endif
/* ************************************************************************* */
TEST(Ordering, Create) {

  // create chain graph
  SymbolicFactorGraph symbolicGraph = example::symbolicChain();

  // COLAMD
  {
    //- P( 4 5)
    //| - P( 3 | 4)
    //| | - P( 2 | 3)
    //| | | - P( 1 | 2)
    //| | | | - P( 0 | 1)
    Ordering actual = Ordering::Create(Ordering::COLAMD, symbolicGraph);
    Ordering expected = Ordering({0, 1, 2, 3, 4, 5});
    EXPECT(assert_equal(expected, actual));
  }

#ifdef GTSAM_SUPPORT_NESTED_DISSECTION
  // METIS
  {
    Ordering actual = Ordering::Create(Ordering::METIS, symbolicGraph);
    //- P( 1 0 2)
    //| - P( 3 4 | 2)
    //| | - P( 5 | 4)
    Ordering expected = Ordering({5, 3, 4, 1, 0, 2});
    EXPECT(assert_equal(expected, actual));
  }
#endif

  // CUSTOM
  CHECK_EXCEPTION(Ordering::Create(Ordering::CUSTOM, symbolicGraph), runtime_error);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
