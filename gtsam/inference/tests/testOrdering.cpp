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
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/symbolic/SymbolicFactorGraph.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/base/TestableAssertions.h>
#include <CppUnitLite/TestHarness.h>

#include <boost/assign/list_of.hpp>

using namespace std;
using namespace gtsam;
using namespace boost::assign;

/* ************************************************************************* */
TEST(Ordering, constrained_ordering) {
  SymbolicFactorGraph sfg;

  // create graph with wanted variable set = 2, 4
  sfg.push_factor(0,1);
  sfg.push_factor(1,2);
  sfg.push_factor(2,3);
  sfg.push_factor(3,4);
  sfg.push_factor(4,5);

  // unconstrained version
  Ordering actUnconstrained = Ordering::COLAMD(sfg);
  Ordering expUnconstrained = Ordering(list_of(0)(1)(2)(3)(4)(5));
  EXPECT(assert_equal(expUnconstrained, actUnconstrained));

  // constrained version - push one set to the end
  Ordering actConstrained = Ordering::COLAMDConstrainedLast(sfg, list_of(2)(4));
  Ordering expConstrained = Ordering(list_of(0)(1)(5)(3)(4)(2));
  EXPECT(assert_equal(expConstrained, actConstrained));

  // constrained version - push one set to the start
  Ordering actConstrained2 = Ordering::COLAMDConstrainedFirst(sfg, list_of(2)(4));
  Ordering expConstrained2 = Ordering(list_of(2)(4)(0)(1)(3)(5));
  EXPECT(assert_equal(expConstrained2, actConstrained2));
}

/* ************************************************************************* */
TEST(Ordering, grouped_constrained_ordering) {
  SymbolicFactorGraph sfg;

  // create graph with constrained groups:
  // 1: 2, 4
  // 2: 5
  sfg.push_factor(0,1);
  sfg.push_factor(1,2);
  sfg.push_factor(2,3);
  sfg.push_factor(3,4);
  sfg.push_factor(4,5);

  // constrained version - push one set to the end
  FastMap<size_t, int> constraints;
  constraints[2] = 1;
  constraints[4] = 1;
  constraints[5] = 2;

  Ordering actConstrained = Ordering::COLAMDConstrained(sfg, constraints);
  Ordering expConstrained = list_of(0)(1)(3)(2)(4)(5);
  EXPECT(assert_equal(expConstrained, actConstrained));
}

/* ************************************************************************* */
int main() { TestResult tr; return TestRegistry::runAllTests(tr); }
/* ************************************************************************* */
