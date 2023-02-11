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

typedef Cyclic<3> Z3; // Let's use the cyclic group of order 3
typedef Cyclic<2> Z2;

//******************************************************************************
TEST(Cyclic, Concept) {
  GTSAM_CONCEPT_ASSERT(IsGroup<Z3>)
  EXPECT_LONGS_EQUAL(0, traits<Z3>::Identity());
}

//******************************************************************************
TEST(Cyclic, Constructor) {
  Z3 g(0);
}

//******************************************************************************
TEST(Cyclic, Compose) {
  EXPECT_LONGS_EQUAL(0, traits<Z3>::Compose(Z3(0),Z3(0)));
  EXPECT_LONGS_EQUAL(1, traits<Z3>::Compose(Z3(0),Z3(1)));
  EXPECT_LONGS_EQUAL(2, traits<Z3>::Compose(Z3(0),Z3(2)));

  EXPECT_LONGS_EQUAL(2, traits<Z3>::Compose(Z3(2),Z3(0)));
  EXPECT_LONGS_EQUAL(0, traits<Z3>::Compose(Z3(2),Z3(1)));
  EXPECT_LONGS_EQUAL(1, traits<Z3>::Compose(Z3(2),Z3(2)));
}

//******************************************************************************
TEST(Cyclic, Between) {
  EXPECT_LONGS_EQUAL(0, traits<Z3>::Between(Z3(0),Z3(0)));
  EXPECT_LONGS_EQUAL(1, traits<Z3>::Between(Z3(0),Z3(1)));
  EXPECT_LONGS_EQUAL(2, traits<Z3>::Between(Z3(0),Z3(2)));

  EXPECT_LONGS_EQUAL(1, traits<Z3>::Between(Z3(2),Z3(0)));
  EXPECT_LONGS_EQUAL(2, traits<Z3>::Between(Z3(2),Z3(1)));
  EXPECT_LONGS_EQUAL(0, traits<Z3>::Between(Z3(2),Z3(2)));
}

//******************************************************************************
TEST(Cyclic, Inverse) {
  EXPECT_LONGS_EQUAL(0, traits<Z3>::Inverse(Z3(0)));
  EXPECT_LONGS_EQUAL(2, traits<Z3>::Inverse(Z3(1)));
  EXPECT_LONGS_EQUAL(1, traits<Z3>::Inverse(Z3(2)));
}

//******************************************************************************
TEST(Cyclic, Negation) {
  EXPECT_LONGS_EQUAL(0, -Z3(0));
  EXPECT_LONGS_EQUAL(2, -Z3(1));
  EXPECT_LONGS_EQUAL(1, -Z3(2));
}

//******************************************************************************
TEST(Cyclic, Negation2) {
  EXPECT_LONGS_EQUAL(0, -Z2(0));
  EXPECT_LONGS_EQUAL(1, -Z2(1));
}

//******************************************************************************
TEST(Cyclic , Invariants) {
  Z3 g(2), h(1);
  EXPECT(check_group_invariants(g,h));
}

//******************************************************************************
// The Direct sum of Z2 and Z2 is *not* Cyclic<4>, but the
// smallest non-cyclic group called the Klein four-group:
typedef DirectSum<Z2, Z2> K4;

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
  // The Direct sum of Z2 and Z2 is *not* Cyclic<4>, but the
  // smallest non-cyclic group called the Klein four-group:
  GTSAM_CONCEPT_ASSERT(IsGroup<K4>)
  GTSAM_CONCEPT_ASSERT(IsTestable<K4>)

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

