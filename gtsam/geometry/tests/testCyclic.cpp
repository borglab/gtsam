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

typedef Cyclic<3> G; // Let's use the cyclic group of order 3

//******************************************************************************
TEST(Cyclic, Concept) {
  BOOST_CONCEPT_ASSERT((IsGroup<G>));
  EXPECT_LONGS_EQUAL(0, traits_x<G>::Identity());
}

//******************************************************************************
TEST(Cyclic, Constructor) {
  G g(0);
}

//******************************************************************************
TEST(Cyclic, Compose) {
  EXPECT_LONGS_EQUAL(0, traits_x<G>::Compose(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, traits_x<G>::Compose(G(0),G(1)));
  EXPECT_LONGS_EQUAL(2, traits_x<G>::Compose(G(0),G(2)));

  EXPECT_LONGS_EQUAL(2, traits_x<G>::Compose(G(2),G(0)));
  EXPECT_LONGS_EQUAL(0, traits_x<G>::Compose(G(2),G(1)));
  EXPECT_LONGS_EQUAL(1, traits_x<G>::Compose(G(2),G(2)));
}

//******************************************************************************
TEST(Cyclic, Between) {
  EXPECT_LONGS_EQUAL(0, traits_x<G>::Between(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, traits_x<G>::Between(G(0),G(1)));
  EXPECT_LONGS_EQUAL(2, traits_x<G>::Between(G(0),G(2)));

  EXPECT_LONGS_EQUAL(1, traits_x<G>::Between(G(2),G(0)));
  EXPECT_LONGS_EQUAL(2, traits_x<G>::Between(G(2),G(1)));
  EXPECT_LONGS_EQUAL(0, traits_x<G>::Between(G(2),G(2)));
}

//******************************************************************************
TEST(Cyclic, Ivnverse) {
  EXPECT_LONGS_EQUAL(0, traits_x<G>::Inverse(G(0)));
  EXPECT_LONGS_EQUAL(2, traits_x<G>::Inverse(G(1)));
  EXPECT_LONGS_EQUAL(1, traits_x<G>::Inverse(G(2)));
}

//******************************************************************************
TEST(Cyclic , Invariants) {
  G g(2), h(5);
 check_group_invariants(g,h);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

