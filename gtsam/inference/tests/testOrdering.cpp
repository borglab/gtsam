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

#include <boost/assign/std.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

namespace example {
SymbolicFactorGraph symbolicChain() {
  SymbolicFactorGraph sfg;
  sfg.push_factor(0, 1);
  sfg.push_factor(1, 2);
  sfg.push_factor(2, 3);
  sfg.push_factor(3, 4);
  sfg.push_factor(4, 5);
  return sfg;
}
}
/* ************************************************************************* */
TEST(Ordering, constrained_ordering) {

  // create graph with wanted variable set = 2, 4
  SymbolicFactorGraph sfg = example::symbolicChain();

  // unconstrained version
  Ordering actUnconstrained = Ordering::Colamd(sfg);
  Ordering expUnconstrained = Ordering(list_of(0)(1)(2)(3)(4)(5));
  EXPECT(assert_equal(expUnconstrained, actUnconstrained));

  // constrained version - push one set to the end
  Ordering actConstrained = Ordering::ColamdConstrainedLast(sfg, list_of(2)(4));
  Ordering expConstrained = Ordering(list_of(0)(1)(5)(3)(4)(2));
  EXPECT(assert_equal(expConstrained, actConstrained));

  // constrained version - push one set to the start
  Ordering actConstrained2 = Ordering::ColamdConstrainedFirst(sfg,
      list_of(2)(4));
  Ordering expConstrained2 = Ordering(list_of(2)(4)(0)(1)(3)(5));
  EXPECT(assert_equal(expConstrained2, actConstrained2));
}

/* ************************************************************************* */
TEST(Ordering, grouped_constrained_ordering) {

  // create graph with constrained groups:
  // 1: 2, 4
  // 2: 5
  SymbolicFactorGraph sfg = example::symbolicChain();

  // constrained version - push one set to the end
  FastMap<size_t, int> constraints;
  constraints[2] = 1;
  constraints[4] = 1;
  constraints[5] = 2;

  Ordering actConstrained = Ordering::ColamdConstrained(sfg, constraints);
  Ordering expConstrained = list_of(0)(1)(3)(2)(4)(5);
  EXPECT(assert_equal(expConstrained, actConstrained));
}

/* ************************************************************************* */
TEST(Ordering, csr_format) {
  // Example in METIS manual
  SymbolicFactorGraph sfg;
  sfg.push_factor(0, 1);
  sfg.push_factor(1, 2);
  sfg.push_factor(2, 3);
  sfg.push_factor(3, 4);
  sfg.push_factor(5, 6);
  sfg.push_factor(6, 7);
  sfg.push_factor(7, 8);
  sfg.push_factor(8, 9);
  sfg.push_factor(10, 11);
  sfg.push_factor(11, 12);
  sfg.push_factor(12, 13);
  sfg.push_factor(13, 14);

  sfg.push_factor(0, 5);
  sfg.push_factor(5, 10);
  sfg.push_factor(1, 6);
  sfg.push_factor(6, 11);
  sfg.push_factor(2, 7);
  sfg.push_factor(7, 12);
  sfg.push_factor(3, 8);
  sfg.push_factor(8, 13);
  sfg.push_factor(4, 9);
  sfg.push_factor(9, 14);

  MetisIndex mi(sfg);

  vector<int> xadjExpected, adjExpected;
  xadjExpected += 0, 2, 5, 8, 11, 13, 16, 20, 24, 28, 31, 33, 36, 39, 42, 44;
  adjExpected += 1, 5, 0, 2, 6, 1, 3, 7, 2, 4, 8, 3, 9, 0, 6, 10, 1, 5, 7, 11, 2, 6, 8, 12, 3, 7, 9, 13, 4, 8, 14, 5, 11, 6, 10, 12, 7, 11, 13, 8, 12, 14, 9, 13;

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == mi.adj());
}

/* ************************************************************************* */
TEST(Ordering, csr_format_2) {
  SymbolicFactorGraph sfg;

  sfg.push_factor(0);
  sfg.push_factor(0, 1);
  sfg.push_factor(1, 2);
  sfg.push_factor(2, 3);
  sfg.push_factor(3, 4);
  sfg.push_factor(4, 1);

  MetisIndex mi(sfg);

  vector<int> xadjExpected, adjExpected;
  xadjExpected += 0, 1, 4, 6, 8, 10;
  adjExpected += 1, 0, 2, 4, 1, 3, 2, 4, 1, 3;

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == mi.adj());
}

/* ************************************************************************* */
TEST(Ordering, csr_format_3) {
  SymbolicFactorGraph sfg;

  sfg.push_factor(100);
  sfg.push_factor(100, 101);
  sfg.push_factor(101, 102);
  sfg.push_factor(102, 103);
  sfg.push_factor(103, 104);
  sfg.push_factor(104, 101);

  MetisIndex mi(sfg);

  vector<int> xadjExpected, adjExpected;
  xadjExpected += 0, 1, 4, 6, 8, 10;
  adjExpected += 1, 0, 2, 4, 1, 3, 2, 4, 1, 3;
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
TEST(Ordering, csr_format_4) {
  SymbolicFactorGraph sfg;

  sfg.push_factor(Symbol('x', 1));
  sfg.push_factor(Symbol('x', 1), Symbol('x', 2));
  sfg.push_factor(Symbol('x', 2), Symbol('x', 3));
  sfg.push_factor(Symbol('x', 3), Symbol('x', 4));
  sfg.push_factor(Symbol('x', 4), Symbol('x', 5));
  sfg.push_factor(Symbol('x', 5), Symbol('x', 6));

  MetisIndex mi(sfg);

  vector<int> xadjExpected, adjExpected;
  xadjExpected += 0, 1, 3, 5, 7, 9, 10;
  adjExpected += 1, 0, 2, 1, 3, 2, 4, 3, 5, 4;

  vector<int> adjAcutal = mi.adj();
  vector<int> xadjActual = mi.xadj();

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == adjAcutal);

  Ordering metOrder = Ordering::Metis(sfg);

  // Test different symbol types
  sfg.push_factor(Symbol('l', 1));
  sfg.push_factor(Symbol('x', 1), Symbol('l', 1));
  sfg.push_factor(Symbol('x', 2), Symbol('l', 1));
  sfg.push_factor(Symbol('x', 3), Symbol('l', 1));
  sfg.push_factor(Symbol('x', 4), Symbol('l', 1));

  Ordering metOrder2 = Ordering::Metis(sfg);
}

/* ************************************************************************* */
TEST(Ordering, metis) {

  SymbolicFactorGraph sfg;

  sfg.push_factor(0);
  sfg.push_factor(0, 1);
  sfg.push_factor(1, 2);

  MetisIndex mi(sfg);

  vector<int> xadjExpected, adjExpected;
  xadjExpected += 0, 1, 3, 4;
  adjExpected += 1, 0, 2, 1;

  EXPECT(xadjExpected == mi.xadj());
  EXPECT(adjExpected.size() == mi.adj().size());
  EXPECT(adjExpected == mi.adj());

  Ordering metis = Ordering::Metis(sfg);
}

/* ************************************************************************* */
TEST(Ordering, MetisLoop) {

  // create linear graph
  SymbolicFactorGraph sfg = example::symbolicChain();

  // add loop closure
  sfg.push_factor(0, 5);

  // METIS
#if !defined(__APPLE__)
  {
    Ordering actual = Ordering::Create(Ordering::METIS, sfg);
    //  - P( 0 4 1)
    //  | - P( 2 | 4 1)
    //  | | - P( 3 | 4 2)
    //  | - P( 5 | 0 1)
    Ordering expected = Ordering(list_of(3)(2)(5)(0)(4)(1));
    EXPECT(assert_equal(expected, actual));
  }
#else
  {
    Ordering actual = Ordering::Create(Ordering::METIS, sfg);
    //  - P( 1 0 3)
    //  | - P( 4 | 0 3)
    //  | | - P( 5 | 0 4)
    //  | - P( 2 | 1 3)
    Ordering expected = Ordering(list_of(5)(4)(2)(1)(0)(3));
    EXPECT(assert_equal(expected, actual));
  }
#endif
}

/* ************************************************************************* */
TEST(Ordering, Create) {

  // create chain graph
  SymbolicFactorGraph sfg = example::symbolicChain();

  // COLAMD
  {
    //- P( 4 5)
    //| - P( 3 | 4)
    //| | - P( 2 | 3)
    //| | | - P( 1 | 2)
    //| | | | - P( 0 | 1)
    Ordering actual = Ordering::Create(Ordering::COLAMD, sfg);
    Ordering expected = Ordering(list_of(0)(1)(2)(3)(4)(5));
    EXPECT(assert_equal(expected, actual));
  }

  // METIS
  {
    Ordering actual = Ordering::Create(Ordering::METIS, sfg);
    //- P( 1 0 2)
    //| - P( 3 4 | 2)
    //| | - P( 5 | 4)
    Ordering expected = Ordering(list_of(5)(3)(4)(1)(0)(2));
    EXPECT(assert_equal(expected, actual));
  }

  // CUSTOM
  CHECK_EXCEPTION(Ordering::Create(Ordering::CUSTOM, sfg), runtime_error);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
