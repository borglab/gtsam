/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testCyclic.cpp
 * @brief  Unit tests for cyclic group
 * @author Frank Dellaert
 **/

#include <gtsam/geometry/Cyclic.h>
#include <gtsam/base/Testable.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

typedef Cyclic<6> G; // Let's use the cyclic group of order 6

//******************************************************************************
TEST(Cyclic, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<G>));
//  BOOST_CONCEPT_ASSERT((IsGroup<AdditiveGroup<G> >));
  EXPECT_LONGS_EQUAL(0, group::traits::identity<G>::value);
  G g(2), h(3);
  // EXPECT(Group<G>().check_invariants(g,h))
}

//******************************************************************************
TEST(Cyclic, Constructor) {
  G g(0);
}

//******************************************************************************
TEST(Cyclic, Compose) {
  EXPECT_LONGS_EQUAL(0, group::compose(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, group::compose(G(0),G(0)));
  EXPECT_LONGS_EQUAL(2, group::compose(G(0),G(2)));
  EXPECT_LONGS_EQUAL(3, group::compose(G(0),G(3)));
  EXPECT_LONGS_EQUAL(4, group::compose(G(0),G(4)));
  EXPECT_LONGS_EQUAL(5, group::compose(G(0),G(5)));

  EXPECT_LONGS_EQUAL(2, group::compose(G(2),G(0)));
  EXPECT_LONGS_EQUAL(3, group::compose(G(2),G(1)));
  EXPECT_LONGS_EQUAL(4, group::compose(G(2),G(2)));
  EXPECT_LONGS_EQUAL(5, group::compose(G(2),G(3)));
  EXPECT_LONGS_EQUAL(0, group::compose(G(2),G(4)));
  EXPECT_LONGS_EQUAL(1, group::compose(G(2),G(5)));
}

//******************************************************************************
TEST(Cyclic, Between) {
  EXPECT_LONGS_EQUAL(0, group::between(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, group::between(G(0),G(1)));
  EXPECT_LONGS_EQUAL(2, group::between(G(0),G(2)));
  EXPECT_LONGS_EQUAL(3, group::between(G(0),G(3)));
  EXPECT_LONGS_EQUAL(4, group::between(G(0),G(4)));
  EXPECT_LONGS_EQUAL(5, group::between(G(0),G(5)));

  EXPECT_LONGS_EQUAL(4, group::between(G(2),G(0)));
  EXPECT_LONGS_EQUAL(5, group::between(G(2),G(1)));
  EXPECT_LONGS_EQUAL(0, group::between(G(2),G(2)));
  EXPECT_LONGS_EQUAL(1, group::between(G(2),G(3)));
  EXPECT_LONGS_EQUAL(2, group::between(G(2),G(4)));
  EXPECT_LONGS_EQUAL(3, group::between(G(2),G(5)));
}

//******************************************************************************
TEST(Cyclic, Ivnverse) {
  EXPECT_LONGS_EQUAL(0, group::inverse(G(0)));
  EXPECT_LONGS_EQUAL(5, group::inverse(G(1)));
  EXPECT_LONGS_EQUAL(4, group::inverse(G(2)));
  EXPECT_LONGS_EQUAL(3, group::inverse(G(3)));
  EXPECT_LONGS_EQUAL(2, group::inverse(G(4)));
  EXPECT_LONGS_EQUAL(1, group::inverse(G(5)));
}

//******************************************************************************
TEST(Cyclic , Invariants) {
  G g(2), h(5);
  group::check_invariants(g,h);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

