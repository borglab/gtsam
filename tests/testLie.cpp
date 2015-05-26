/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------1------------------------------------------- */

/**
 * @file testLie.cpp
 * @date May, 2015
 * @author Frank Dellaert
 * @brief unit tests for Lie group type machinery
 */

#include <gtsam/base/ProductLieGroup.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/base/testLie.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

//******************************************************************************
typedef ProductLieGroup<Point2, Point2> MyPoint2Pair;

// Define any direct product group to be a model of the multiplicative Group concept
namespace gtsam {
template<> struct traits<MyPoint2Pair> : internal::LieGroupTraits<MyPoint2Pair> {
  static void Print(const MyPoint2Pair& m, const string& s = "") {
    cout << s << "(" << m.first << "," << m.second << ")" << endl;
  }
  static bool Equals(const MyPoint2Pair& m1, const MyPoint2Pair& m2,
      double tol = 1e-8) {
    return m1 == m2;
  }
};
}

TEST(Lie, ProductLieGroup) {
  BOOST_CONCEPT_ASSERT((IsGroup<MyPoint2Pair>));
  BOOST_CONCEPT_ASSERT((IsManifold<MyPoint2Pair>));
  BOOST_CONCEPT_ASSERT((IsLieGroup<MyPoint2Pair>));
  MyPoint2Pair pair1;
  Vector4 d;
  d << 1, 2, 3, 4;
  MyPoint2Pair expected(Point2(1, 2), Point2(3, 4));
  MyPoint2Pair pair2 = pair1.retract(d);
  EXPECT(assert_equal(expected, pair2, 1e-9));
  EXPECT(assert_equal(d, pair1.localCoordinates(pair2), 1e-9));
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************

