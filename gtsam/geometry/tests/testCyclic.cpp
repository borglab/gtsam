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
  EXPECT_LONGS_EQUAL(0, traits<G>::Identity());
}

//******************************************************************************
TEST(Cyclic, Constructor) {
  G g(0);
}

//******************************************************************************
TEST(Cyclic, Compose) {
  EXPECT_LONGS_EQUAL(0, traits<G>::Compose(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Compose(G(0),G(1)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Compose(G(0),G(2)));

  EXPECT_LONGS_EQUAL(2, traits<G>::Compose(G(2),G(0)));
  EXPECT_LONGS_EQUAL(0, traits<G>::Compose(G(2),G(1)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Compose(G(2),G(2)));
}

//******************************************************************************
TEST(Cyclic, Between) {
  EXPECT_LONGS_EQUAL(0, traits<G>::Between(G(0),G(0)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Between(G(0),G(1)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Between(G(0),G(2)));

  EXPECT_LONGS_EQUAL(1, traits<G>::Between(G(2),G(0)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Between(G(2),G(1)));
  EXPECT_LONGS_EQUAL(0, traits<G>::Between(G(2),G(2)));
}

//******************************************************************************
TEST(Cyclic, Inverse) {
  EXPECT_LONGS_EQUAL(0, traits<G>::Inverse(G(0)));
  EXPECT_LONGS_EQUAL(2, traits<G>::Inverse(G(1)));
  EXPECT_LONGS_EQUAL(1, traits<G>::Inverse(G(2)));
}

//******************************************************************************
TEST(Cyclic, Negation) {
  EXPECT_LONGS_EQUAL(0, -G(0));
  EXPECT_LONGS_EQUAL(2, -G(1));
  EXPECT_LONGS_EQUAL(1, -G(2));
}

//******************************************************************************
TEST(Cyclic, Negation2) {
  typedef Cyclic<2> Z2;
  EXPECT_LONGS_EQUAL(0, -Z2(0));
  EXPECT_LONGS_EQUAL(1, -Z2(1));
}

//******************************************************************************
TEST(Cyclic , Invariants) {
  G g(2), h(1);
  EXPECT(check_group_invariants(g,h));
}

//******************************************************************************
// The Direct sum of Cyclic<2> and Cyclic<2> is *not* Cyclic<4>, but the
// smallest non-cyclic group called the Klein four-group:
typedef DirectSum<Cyclic<2>, Cyclic<2> > K4;

namespace gtsam {

/// Define K4 to be a model of the Additive Group concept, and provide Testable
template<>
struct traits<K4> : internal::AdditiveGroupTraits<K4> {
  static void Print(const K4& m, const string& s = "") {
    cout << s << "(" << m.first << "," << m.second << ")" << endl;
  }
  static bool Equals(const K4& m1, const K4& m2, double tol = 1e-8) {
    return m1 == m2;
  }
};

}  // namespace gtsam

TEST(Cyclic , DirectSum) {
  // The Direct sum of Cyclic<2> and Cyclic<2> is *not* Cyclic<4>, but the
  // smallest non-cyclic group called the Klein four-group:
  typedef DirectSum<Cyclic<2>, Cyclic<2> > K4;
  BOOST_CONCEPT_ASSERT((IsGroup<K4>));
  BOOST_CONCEPT_ASSERT((IsTestable<K4>));

  // Refer to http://en.wikipedia.org/wiki/Klein_four-group
  K4 e(0,0), a(0, 1), b(1, 0), c(1, 1);
  EXPECT(assert_equal(a, - a));
  EXPECT(assert_equal(b, - b));
  EXPECT(assert_equal(c, - c));
  EXPECT(assert_equal(a, a + e));
  EXPECT(assert_equal(b, b + e));
  EXPECT(assert_equal(c, c + e));
  EXPECT(assert_equal(e, a + a));
  EXPECT(assert_equal(e, b + b));
  EXPECT(assert_equal(e, c + c));
  EXPECT(assert_equal(c, a + b));
  EXPECT(assert_equal(b, a + c));
  EXPECT(assert_equal(a, b + c));
  EXPECT(assert_equal(c, a - b));
  EXPECT(assert_equal(a, b - c));
  EXPECT(assert_equal(b, c - a));
  EXPECT(check_group_invariants(a, b));
  EXPECT(check_group_invariants(b, c));
  EXPECT(check_group_invariants(c, a));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

